#!/usr/bin/env python3
"""PatchCore v3 offline/runtime 공간 점수와 정책을 CUDA 없이 검증합니다."""

from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np
import torch

WORKSPACE = Path(__file__).parents[1]
REPOSITORY = WORKSPACE.parent
for package_path in (
    REPOSITORY / "offline_model_tools",
    WORKSPACE / "src" / "basic_packages" / "inspection_common",
    WORKSPACE / "src" / "nodes" / "inspection_vision",
):
    sys.path.insert(0, str(package_path))

from inspection_vision.model_backend import (  # noqa: E402
    ArtifactContractError,
    Mono8PatchCorePreprocessor,
    PatchCoreArtifactModel,
    PatchCoreViewModel as RuntimePatchCoreViewModel,
    PreprocessingSettings,
    spatial_view_scores,
)
from patchcore_v3_common import (  # noqa: E402
    aggregate_maps_numpy,
    build_spatial_calibration,
    find_product_fpr_thresholds,
    normalize_maps_numpy,
    preprocess_full_frame,
    spatial_view_scores_torch,
    top_k_percent_patch_count,
    validate_aggregation_config,
)
from ad_common import PatchCoreViewModel as OfflinePatchCoreViewModel  # noqa: E402
from MB_construction_2 import MB_CONFIGS, validate_config  # noqa: E402
from patchcore_AD_2 import (  # noqa: E402
    _classification_metrics,
    _save_normalized_heatmap,
    _selected_policy_document,
)
from patchcore_v3_common import native_patch_maps  # noqa: E402


SERIAL_TO_VIEW = {
    "DA9880512": "CAM_A_1",
    "DA9880516": "CAM_A_2",
    "DA7552836": "CAM_A_3",
    "DA7838410": "CAM_B_1",
}


class PatchCoreV3PolicyTests(unittest.TestCase):
    def test_mb_config_accepts_independent_view_model_settings(self) -> None:
        config = copy.deepcopy(MB_CONFIGS["MB_v3_resol_180"])
        config["parameters_by_view"]["CAM_A_1"].update(
            {"feature_layers": [1], "input_resolution": (160, 192)}
        )
        config["parameters_by_view"]["CAM_A_2"].update(
            {
                "backbone": "efficientnet_b0",
                "feature_layers": [2, 4],
                "input_resolution": (224, 224),
            }
        )
        config["parameters_by_view"]["CAM_A_3"].update(
            {"feature_layers": [2, 3], "input_resolution": (192, 160)}
        )
        config["parameters_by_view"]["CAM_B_1"].update(
            {"feature_layers": [3], "input_resolution": (128, 128)}
        )
        for position, view in enumerate(SERIAL_TO_VIEW.values()):
            config["preprocessing_by_view"][view]["v_threshold"] = 20 + position * 30

        validate_config(config)

    def test_offline_and_runtime_native_patch_maps_are_identical(self) -> None:
        torch.manual_seed(17)
        offline = OfflinePatchCoreViewModel(
            backbone="resnet34",
            layer_numbers=[1],
            num_neighbors=1,
            pretrained=False,
            distance_chunk_size=64,
        ).eval()
        runtime = RuntimePatchCoreViewModel(
            backbone="resnet34",
            layer_numbers=[1],
            num_neighbors=1,
            pretrained=False,
            distance_chunk_size=64,
        ).eval()
        runtime.load_state_dict(offline.state_dict(), strict=True)
        memory_bank = torch.randn((20, 64), dtype=torch.float32)
        offline.memory_bank = memory_bank.clone()
        runtime.memory_bank = memory_bank.clone()
        images = torch.rand((1, 3, 32, 32), dtype=torch.float32)

        expected = native_patch_maps(offline, images)
        actual = runtime.native_patch_maps(images)

        torch.testing.assert_close(actual, expected, rtol=0, atol=0)

    def test_offline_and_runtime_spatial_scores_are_identical(self) -> None:
        rng = np.random.default_rng(42)
        calibration_maps = rng.normal(0.5, 0.1, size=(20, 4, 5)).astype(np.float32)
        raw_maps = rng.normal(0.6, 0.2, size=(3, 4, 5)).astype(np.float32)
        methods = (
            {"method": "epsilon", "epsilon_ratio": 0.01},
            {"method": "std_floor", "std_floor_ratio": 0.1},
            {"method": "shrinkage", "shrinkage_lambda": 0.25},
            {"method": "mad", "mad_epsilon_ratio": 0.01},
        )
        aggregations = (
            {"method": "percentile", "percentile": 99.5},
            {
                "method": "top_k_percent_average",
                "top_k_percent": 10.0,
                "rounding": "ceil",
                "minimum_patch_count": 1,
            },
        )
        for method in methods:
            calibration = build_spatial_calibration(calibration_maps, method)
            normalized = normalize_maps_numpy(raw_maps, calibration)
            for aggregation in aggregations:
                expected = aggregate_maps_numpy(normalized, aggregation)
                offline_tensor = spatial_view_scores_torch(
                    torch.from_numpy(raw_maps),
                    torch.from_numpy(calibration.center),
                    torch.from_numpy(calibration.denominator),
                    aggregation,
                )
                runtime_tensor = spatial_view_scores(
                    torch.from_numpy(raw_maps),
                    torch.from_numpy(calibration.center),
                    torch.from_numpy(calibration.denominator),
                    aggregation,
                )
                np.testing.assert_allclose(offline_tensor.numpy(), expected, rtol=1e-5, atol=1e-6)
                torch.testing.assert_close(runtime_tensor, offline_tensor)

    def test_ratio_top_k_uses_percentage_ceil_and_rejects_absolute_contract(self) -> None:
        self.assertEqual(top_k_percent_patch_count(400, 1.0), 4)
        self.assertEqual(top_k_percent_patch_count(784, 1.0), 8)
        self.assertEqual(top_k_percent_patch_count(201, 1.0), 3)
        self.assertEqual(top_k_percent_patch_count(20, 1.0), 1)
        with self.assertRaisesRegex(ValueError, "지원하지 않는 aggregation"):
            validate_aggregation_config({"method": "top_k_average", "top_k": 3})

    def test_final_metrics_and_policy_summary_are_complete(self) -> None:
        metrics = _classification_metrics((90, 10, 2, 98), 200)
        self.assertEqual(metrics["confusion_matrix"]["matrix"], [[90, 10], [2, 98]])
        self.assertAlmostEqual(metrics["recall"], 0.98)
        self.assertAlmostEqual(metrics["precision"], 98 / 108)
        self.assertAlmostEqual(metrics["fpr"], 0.1)
        self.assertAlmostEqual(metrics["f1"], 196 / 208)

        aggregation = {
            "method": "top_k_percent_average",
            "top_k_percent": 1.0,
            "rounding": "ceil",
            "minimum_patch_count": 1,
        }
        manifest = {
            "candidate_selection": {"selected_candidate_id": "candidate-001"},
            "spatial_scoring_policy": {
                "normalization": {"method": "std_floor", "std_floor_ratio": 0.1},
                "aggregation": aggregation,
                "decision": {"target_product_fpr": 0.01},
            },
            "view_names": ["CAM_A_1", "CAM_B_1"],
            "patch_grid_shapes": {"CAM_A_1": [20, 20], "CAM_B_1": [28, 28]},
            "thresholds": {"CAM_A_1": 2.0, "CAM_B_1": 3.0},
            "parameters_by_view": {"CAM_A_1": {}, "CAM_B_1": {}},
            "preprocessing_by_view": {"CAM_A_1": {}, "CAM_B_1": {}},
        }
        summary = _selected_policy_document(manifest)
        self.assertEqual(
            summary["effective_top_k_patch_counts_by_view"],
            {"CAM_A_1": 4, "CAM_B_1": 8},
        )

    def test_heatmap_and_annotated_overlay_are_both_written(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "CAM_A_1_normalized_heatmap.png"
            _save_normalized_heatmap(
                output,
                torch.linspace(0, 1, 3 * 16 * 16).reshape(3, 16, 16),
                np.asarray([[0.0, 1.0], [2.0, 3.0]], dtype=np.float32),
                view_score=2.0,
                threshold=1.5,
                ratio_score=4.0 / 3.0,
                prediction=True,
            )
            overlay = output.with_name("CAM_A_1_normalized_overlay.png")
            self.assertTrue(output.is_file())
            self.assertTrue(overlay.is_file())
            self.assertIsNotNone(cv2.imread(str(output), cv2.IMREAD_COLOR))
            self.assertIsNotNone(cv2.imread(str(overlay), cv2.IMREAD_COLOR))

    def test_product_threshold_uses_strict_four_view_or_fpr(self) -> None:
        base = np.arange(100, dtype=np.float64)
        scores = {
            "CAM_A_1": base,
            "CAM_A_2": base,
            "CAM_A_3": base,
            "CAM_B_1": base,
        }
        percentile, thresholds, fpr = find_product_fpr_thresholds(scores, 0.01)
        self.assertLessEqual(fpr, 0.01)
        self.assertGreaterEqual(percentile, 98.9)
        predictions = np.zeros(100, dtype=bool)
        for view, values in scores.items():
            predictions |= values > thresholds[view]
        self.assertEqual(float(predictions.mean()), fpr)

    def test_offline_and_runtime_full_frame_preprocessing_match(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = np.zeros((120, 200), dtype=np.uint8)
            image[20:100, 50:150] = 150
            image[2:4, 2:4] = 255
            source = root / "DA9880512.png"
            ok, encoded = cv2.imencode(".png", image)
            self.assertTrue(ok)
            source.write_bytes(encoded.tobytes())
            payload = {
                "v_threshold": 40,
                "connectivity": 8,
                "remove_disconnected_noise": True,
                "check_connection": False,
            }
            offline = preprocess_full_frame(
                source,
                view="CAM_A_1",
                settings=payload,
                resolution=(180, 180),
                resize_mode="padding",
            )
            runtime = Mono8PatchCorePreprocessor(
                serial_to_view=SERIAL_TO_VIEW,
                settings_by_view={
                    view: PreprocessingSettings(40, 8, True, False)
                    for view in SERIAL_TO_VIEW.values()
                },
                input_resolution_by_view={view: (180, 180) for view in SERIAL_TO_VIEW.values()},
                resize_mode_by_view={view: "padding" for view in SERIAL_TO_VIEW.values()},
                diagnostic_root=root / "diagnostics",
            ).load(source)
            torch.testing.assert_close(offline.tensor, runtime.tensor)
            np.testing.assert_array_equal(offline.crop_1, runtime.crop_1)
            np.testing.assert_array_equal(offline.crop_2, runtime.crop_2)

    def test_v2_artifact_is_explicitly_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ArtifactContractError, "format_version"):
                PatchCoreArtifactModel._validate_manifest(
                    Path(directory),
                    {"format_version": 2},
                    expected_version="old-v2",
                    serial_to_view=SERIAL_TO_VIEW,
                )


if __name__ == "__main__":
    unittest.main(verbosity=2)

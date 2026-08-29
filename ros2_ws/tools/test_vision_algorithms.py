"""ROS와 실제 카메라 없이 PatchCore 전처리와 MVS adapter를 검증합니다."""

from __future__ import annotations

import ctypes
import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np
import torch

WORKSPACE = Path(__file__).parents[1]
for package_path in (
    WORKSPACE / "src" / "basic_packages" / "inspection_common",
    WORKSPACE / "src" / "nodes" / "inspection_vision",
):
    sys.path.insert(0, str(package_path))

from inspection_vision.hikrobot_mvs import (  # noqa: E402
    ActionGroup,
    CameraAcquisitionSettings,
    CameraInventoryEntry,
    HikrobotMvsCaptureBackend,
    MvsBackendSettings,
    _DeviceRecord,
    _MvsModules,
)
from inspection_vision.model_backend import (  # noqa: E402
    EXPECTED_STATION_VIEWS,
    EXPECTED_PREPROCESSING_PIPELINE,
    ArtifactContractError,
    Mono8PatchCorePreprocessor,
    PatchCoreArtifactModel,
    PreprocessingFailure,
    PreprocessingSettings,
    artifact_directory_sha256,
)


SERIAL_TO_VIEW = {
    "DA9880512": "CAM_A_1",
    "DA9880516": "CAM_A_2",
    "DA7552836": "CAM_A_3",
    "DA7838410": "CAM_B_1",
}


def write_png(path: Path, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError("test PNG encoding failed")
    path.write_bytes(encoded.tobytes())


class PatchCoreContractTests(unittest.TestCase):
    def _preprocessor(self, root: Path) -> Mono8PatchCorePreprocessor:
        settings = {
            view: PreprocessingSettings(40, 8, True, False)
            for view in SERIAL_TO_VIEW.values()
        }
        return Mono8PatchCorePreprocessor(
            serial_to_view=SERIAL_TO_VIEW,
            settings_by_view=settings,
            input_resolution_by_view={
                "CAM_A_1": (180, 180),
                "CAM_A_2": (160, 192),
                "CAM_A_3": (192, 160),
                "CAM_B_1": (128, 128),
            },
            resize_mode_by_view={view: "padding" for view in SERIAL_TO_VIEW.values()},
            diagnostic_root=root / "diagnostics",
        )

    def test_mono8_foreground_crop_padding_and_tensor_contract(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = np.zeros((120, 200), dtype=np.uint8)
            image[20:100, 50:150] = 150
            image[2:4, 2:4] = 255  # 가장 큰 component만 유지합니다.
            source = root / "DA9880512.png"
            write_png(source, image)

            prepared = self._preprocessor(root).load(source)

            self.assertEqual(prepared.view_name, "CAM_A_1")
            self.assertEqual(prepared.original_component_count, 2)
            self.assertEqual(prepared.crop_1.shape, image.shape)
            self.assertEqual(prepared.crop_2.shape, (80, 100))
            self.assertEqual(tuple(prepared.tensor.shape), (3, 180, 180))
            self.assertEqual(str(prepared.tensor.dtype), "torch.float32")
            self.assertTrue(bool((prepared.tensor[:, 0, :] == 0).all()))
            self.assertFalse((root / "diagnostics").exists())

    def test_each_view_uses_its_own_input_resolution(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = np.zeros((120, 200), dtype=np.uint8)
            image[20:100, 50:150] = 150
            source = root / "DA7838410.png"
            write_png(source, image)

            prepared = self._preprocessor(root).load(source)

            self.assertEqual(prepared.view_name, "CAM_B_1")
            self.assertEqual(tuple(prepared.tensor.shape), (3, 128, 128))

    def test_each_view_uses_its_own_v_threshold(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = np.zeros((80, 80), dtype=np.uint8)
            image[20:60, 20:60] = 150
            source_a = root / "DA9880512.png"
            source_b = root / "DA7838410.png"
            write_png(source_a, image)
            write_png(source_b, image)
            settings = {
                view: PreprocessingSettings(
                    100 if view == "CAM_A_1" else 200,
                    8,
                    True,
                    False,
                )
                for view in SERIAL_TO_VIEW.values()
            }
            preprocessor = Mono8PatchCorePreprocessor(
                serial_to_view=SERIAL_TO_VIEW,
                settings_by_view=settings,
                input_resolution_by_view={
                    view: (64, 64) for view in SERIAL_TO_VIEW.values()
                },
                resize_mode_by_view={
                    view: "padding" for view in SERIAL_TO_VIEW.values()
                },
                diagnostic_root=root / "diagnostics",
            )

            self.assertEqual(preprocessor.load(source_a).view_name, "CAM_A_1")
            with self.assertRaises(PreprocessingFailure):
                preprocessor.load(source_b)

    def test_no_foreground_is_failure_and_saves_only_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "DA7838410.png"
            write_png(source, np.zeros((32, 32), dtype=np.uint8))
            with self.assertRaises(PreprocessingFailure):
                self._preprocessor(root).load(source)
            diagnostics = sorted((root / "diagnostics").rglob("*.png"))
            self.assertEqual(len(diagnostics), 2)
            self.assertTrue(
                all("preprocessing_failure" in path.parts for path in diagnostics)
            )

    def _write_manifest_fixture(self, root: Path) -> tuple[dict, str]:
        views = [
            view
            for station in (1, 2)
            for view in EXPECTED_STATION_VIEWS[station]
        ]
        thresholds = {view: 2.0 for view in views}
        calibration_b = {
            view: [0.5 + position * 0.001 for position in range(100)]
            for view in views
        }
        shapes = {
            "CAM_A_1": [10, 192],
            "CAM_A_2": [11, 256],
            "CAM_A_3": [12, 384],
            "CAM_B_1": [13, 512],
        }
        preprocessing = {
            view: {
                "v_threshold": 20 + position * 30,
                "connectivity": 8,
                "remove_disconnected_noise": True,
                "check_connection": False,
            }
            for position, view in enumerate(views)
        }
        grids = {view: [2 + position, 2] for position, view in enumerate(views)}
        normalization = {"method": "std_floor", "std_floor_ratio": 0.1}
        aggregation = {
            "method": "top_k_percent_average",
            "top_k_percent": 1.0,
            "rounding": "ceil",
            "minimum_patch_count": 1,
        }
        manifest = {
            "format_version": 3,
            "algorithm": "patchcore",
            "artifact_name": "fixture-v3",
            "model_version": "fixture-v3",
            "view_names": views,
            "station_views": {
                "station_a": list(EXPECTED_STATION_VIEWS[1]),
                "station_b": list(EXPECTED_STATION_VIEWS[2]),
            },
            "camera_serial_by_view": {
                view: serial for serial, view in SERIAL_TO_VIEW.items()
            },
            "parameters_by_view": {
                view: {
                    "backbone": "resnet34",
                    "feature_layers": [1, 2 + (position % 2)],
                    "coreset_ratio": 0.1 + position * 0.01,
                    "k": 9 - position,
                    "input_resolution": [180 + position * 8, 176 + position * 4],
                    "resize_mode": "padding",
                    "construction_batch_size": 1 + position,
                    "distance_chunk_size": 1024 * (position + 1),
                    "seed": 42 + position,
                }
                for position, view in enumerate(views)
            },
            "thresholds": thresholds,
            "memory_bank_shapes": shapes,
            "patch_grid_shapes": grids,
            "calibration_b_scores": calibration_b,
            "preprocessing_by_view": preprocessing,
            "preprocessing_pipeline": dict(EXPECTED_PREPROCESSING_PIPELINE),
            "spatial_scoring_policy": {
                "normalization": normalization,
                "aggregation": aggregation,
                "decision": {
                    "target_product_fpr": 0.01,
                    "threshold_percentile": 99.9,
                    "comparison": "strict_greater_than",
                    "view_score_normalization": "divide_by_threshold",
                },
            },
            "candidate_selection": {
                "selected_candidate_id": "candidate-001",
                "candidates": [
                    {
                        "candidate_id": "candidate-001",
                        "status": "valid",
                        "eligible_for_selection": True,
                        "normalization": normalization,
                        "aggregation": aggregation,
                        "thresholds": thresholds,
                        "threshold_percentile": 99.9,
                        "calibration_product_fpr": 0.01,
                        "validation_metrics": {"fpr": 0.01, "recall": 1.0},
                    }
                ],
            },
            "dataset_provenance": {
                split: {"product_count": 100, "file_count": 400, "sha256": "0" * 64}
                for split in (
                    "training_set",
                    "calibration_set_A",
                    "calibration_set_B",
                    "validation_normal",
                    "validation_anomaly",
                )
            },
            "calibration_b_sample_policy": {
                "minimum": 100,
                "recommended": 1000,
                "actual": 100,
            },
            "parallel_benchmark": {
                "selected_parallel_count": 1,
                "reserve_mib": 512,
                "warmup_runs": 3,
                "benchmark_runs": 10,
                "min_speedup_percent": 5.0,
                "candidates": [],
                "all_view_memory_safety": {"memory_safe": True},
            },
            "library_versions": {"torch": "0.0", "torchvision": "0.0"},
        }
        for view in views:
            view_root = root / view
            view_root.mkdir(parents=True)
            (view_root / "model.pt").write_bytes(f"state:{view}".encode())
            grid_h, grid_w = grids[view]
            torch.save(
                {
                    "center": torch.zeros((grid_h, grid_w), dtype=torch.float32),
                    "denominator": torch.ones((grid_h, grid_w), dtype=torch.float32),
                    "raw_scale": torch.ones((grid_h, grid_w), dtype=torch.float32),
                    "grid_shape": torch.tensor(grids[view], dtype=torch.int64),
                    "scale_reference": torch.tensor(1.0, dtype=torch.float64),
                    "sample_count": torch.tensor(100, dtype=torch.int64),
                },
                view_root / "spatial_calibration.pt",
            )
            (view_root / "calibration.json").write_text(
                json.dumps(
                    {
                        "view": view,
                        "threshold": thresholds[view],
                        "memory_bank_shape": shapes[view],
                        "patch_grid_shape": grids[view],
                        "calibration_b_scores": calibration_b[view],
                        "normalization": normalization,
                        "aggregation": aggregation,
                    }
                ),
                encoding="utf-8",
            )
        (root / "manifest.json").write_text(
            json.dumps(manifest, sort_keys=True), encoding="utf-8"
        )
        return manifest, artifact_directory_sha256(root)

    def test_v3_manifest_requires_four_views_and_directory_hash_is_sensitive(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            manifest, first_digest = self._write_manifest_fixture(root)
            station_views = PatchCoreArtifactModel._validate_manifest(
                root,
                manifest,
                expected_version="fixture-v3",
                serial_to_view=SERIAL_TO_VIEW,
            )
            self.assertEqual(station_views, dict(EXPECTED_STATION_VIEWS))
            model_path = root / "CAM_B_1" / "model.pt"
            model_path.write_bytes(model_path.read_bytes() + b"changed")
            self.assertNotEqual(first_digest, artifact_directory_sha256(root))

            invalid = dict(manifest)
            invalid["view_names"] = list(manifest["view_names"][:-1])
            with self.assertRaises(ArtifactContractError):
                PatchCoreArtifactModel._validate_manifest(
                    root,
                    invalid,
                    expected_version="fixture-v3",
                    serial_to_view=SERIAL_TO_VIEW,
                )


class _FrameInfo(ctypes.Structure):
    _fields_ = [
        ("nWidth", ctypes.c_uint32),
        ("nExtendWidth", ctypes.c_uint32),
        ("nHeight", ctypes.c_uint32),
        ("nExtendHeight", ctypes.c_uint32),
        ("enPixelType", ctypes.c_uint32),
        ("nFrameLenEx", ctypes.c_uint32),
        ("nFrameLen", ctypes.c_uint32),
        ("nLostPacket", ctypes.c_uint32),
        ("nDevTimeStampHigh", ctypes.c_uint32),
        ("nDevTimeStampLow", ctypes.c_uint32),
    ]


class _FrameOut(ctypes.Structure):
    _fields_ = [("pBufAddr", ctypes.c_void_p), ("stFrameInfo", _FrameInfo)]


class _NetDetect(ctypes.Structure):
    _fields_ = [
        ("nLostPacketCount", ctypes.c_uint32),
        ("nRequestResendPacketCount", ctypes.c_uint32),
        ("nResendPacketCount", ctypes.c_uint32),
    ]


class _AllMatch(ctypes.Structure):
    _fields_ = [
        ("nType", ctypes.c_uint32),
        ("pInfo", ctypes.c_void_p),
        ("nInfoSize", ctypes.c_uint32),
    ]


class _ActionCommand(ctypes.Structure):
    _fields_ = [
        ("nDeviceKey", ctypes.c_uint32),
        ("nGroupKey", ctypes.c_uint32),
        ("nGroupMask", ctypes.c_uint32),
        ("bActionTimeEnable", ctypes.c_uint8),
        ("nActionTime", ctypes.c_uint64),
        ("pBroadcastAddress", ctypes.c_char_p),
        ("nTimeOut", ctypes.c_uint32),
        ("bSpecialNetEnable", ctypes.c_uint8),
    ]


class _ActionResult(ctypes.Structure):
    _fields_ = [("strDeviceAddress", ctypes.c_char * 16), ("nStatus", ctypes.c_uint32)]


class _ActionResultList(ctypes.Structure):
    _fields_ = [
        ("nNumResults", ctypes.c_uint32),
        ("pResults", ctypes.POINTER(_ActionResult)),
    ]


class _FakeCamera:
    configured: list[tuple[str, str, object]] = []
    inaccessible_bool_nodes: set[str] = set()
    _action_results = None

    @staticmethod
    def MV_CC_GetSDKVersion() -> int:
        return 0x05000200

    @staticmethod
    def MV_GIGE_IssueActionCommand(command, results) -> int:
        address = b"192.168.10.12" if command.nGroupKey == 2 else b"192.168.10.13"
        values = (_ActionResult * 1)()
        values[0].strDeviceAddress = address
        values[0].nStatus = 0
        _FakeCamera._action_results = values
        results.nNumResults = 1
        results.pResults = ctypes.cast(values, ctypes.POINTER(_ActionResult))
        return 0

    def __init__(self) -> None:
        self._pixels = None
        self._bool_values: dict[str, bool] = {}

    def MV_CC_CreateHandle(self, _raw) -> int:
        return 0

    def MV_CC_OpenDevice(self, *_args) -> int:
        return 0

    def MV_CC_SetEnumValueByString(self, name, value) -> int:
        self.configured.append(("enum", name, value))
        return 0

    def MV_CC_SetIntValueEx(self, name, value) -> int:
        self.configured.append(("int", name, value))
        return 0

    def MV_CC_SetFloatValue(self, name, value) -> int:
        self.configured.append(("float", name, value))
        return 0

    def MV_CC_SetBoolValue(self, name, value) -> int:
        self.configured.append(("bool", name, value))
        if name in self.inaccessible_bool_nodes:
            return 0x80000106
        self._bool_values[name] = bool(value)
        return 0

    def MV_CC_GetBoolValue(self, name, value) -> int:
        if name in self.inaccessible_bool_nodes:
            return 0x80000106
        value.value = self._bool_values.get(name, False)
        return 0

    def MV_GIGE_SetResend(self, *_args) -> int:
        return 0

    def MV_GIGE_SetResendMaxRetryTimes(self, *_args) -> int:
        return 0

    def MV_GIGE_SetResendTimeInterval(self, *_args) -> int:
        return 0

    def MV_CC_SetImageNodeNum(self, *_args) -> int:
        return 0

    def MV_CC_SetGrabStrategy(self, *_args) -> int:
        return 0x80000001

    def MV_CC_StartGrabbing(self) -> int:
        return 0

    def MV_CC_StopGrabbing(self) -> int:
        return 0

    def MV_CC_CloseDevice(self) -> int:
        return 0

    def MV_CC_DestroyHandle(self) -> int:
        return 0

    def MV_CC_GetImageBuffer(self, frame, timeout) -> int:
        if timeout == 1:
            return 0x80000007
        size = 2448 * 2048
        self._pixels = (ctypes.c_uint8 * size)()
        frame.pBufAddr = ctypes.cast(self._pixels, ctypes.c_void_p)
        frame.stFrameInfo.nWidth = 2448
        frame.stFrameInfo.nHeight = 2048
        frame.stFrameInfo.enPixelType = 0x01080001
        frame.stFrameInfo.nFrameLenEx = size
        frame.stFrameInfo.nDevTimeStampLow = 123
        return 0

    def MV_CC_FreeImageBuffer(self, _frame) -> int:
        return 0

    def MV_CC_GetAllMatchInfo(self, _query) -> int:
        return 0


class MvsFakeSdkTests(unittest.TestCase):
    def test_fake_sdk_initialization_action_capture_and_corrections_off(self) -> None:
        serials = ("DA9880512", "DA9880516", "DA7552836", "DA7838410")
        addresses = (
            "192.168.10.13",
            "192.168.10.11",
            "192.168.10.14",
            "192.168.10.12",
        )
        records = tuple(
            _DeviceRecord(
                CameraInventoryEntry(
                    serial,
                    1 if index < 3 else 2,
                    addresses[index],
                    "MV-CS050-10GC",
                    "V1.0",
                ),
                object(),
            )
            for index, serial in enumerate(serials)
        )
        params = types.SimpleNamespace(
            MV_ACTION_CMD_INFO=_ActionCommand,
            MV_ACTION_CMD_RESULT_LIST=_ActionResultList,
            MV_FRAME_OUT=_FrameOut,
            MV_MATCH_INFO_NET_DETECT=_NetDetect,
            MV_ALL_MATCH_INFO=_AllMatch,
            MV_GrabStrategy_OneByOne=0,
        )
        modules = _MvsModules(
            camera=types.SimpleNamespace(MvCamera=_FakeCamera),
            params=params,
            constants=types.SimpleNamespace(
                MV_ACCESS_Exclusive=1,
                MV_MATCH_TYPE_NET_DETECT=1,
            ),
            pixels=types.SimpleNamespace(PixelType_Gvsp_Mono8=0x01080001),
            errors=types.SimpleNamespace(
                MV_E_NODATA=0x80000007,
                MV_E_SUPPORT=0x80000001,
                MV_E_GC_ACCESS=0x80000106,
            ),
        )
        _FakeCamera.configured.clear()
        _FakeCamera.inaccessible_bool_nodes = {"SaturationEnable"}
        with tempfile.TemporaryDirectory() as directory:
            settings = MvsBackendSettings(
                station_camera_ids={1: serials[:3], 2: serials[3:]},
                camera_network_map=dict(zip(serials, addresses, strict=True)),
                camera_acquisition_settings={
                    serial: CameraAcquisitionSettings(
                        10000.0 if serial == "DA7838410" else 5000.0,
                        0.0,
                    )
                    for serial in serials
                },
                action_device_key=1,
                action_groups={1: ActionGroup(1, 1), 2: ActionGroup(2, 2)},
                data_root=Path(directory),
                acquisition_timeout_ms=250,
            )
            backend = HikrobotMvsCaptureBackend(
                settings, sdk_loader=lambda _import, _runtime: modules
            )
            with patch(
                "inspection_vision.hikrobot_mvs._enumerate_devices",
                return_value=records,
            ):
                backend.initialize()
                batch = backend.capture_station(
                    product_id="product-1",
                    station_id=2,
                    capture_id="capture-1",
                    attempt=1,
                    required_camera_ids=("DA7838410",),
                )
            self.assertEqual(len(batch.images), 1)
            self.assertEqual(batch.images[0].camera_id, "DA7838410")
            self.assertEqual(batch.images[0].packet_loss_count, 0)
            self.assertTrue(Path(batch.images[0].file_path).is_file())
            self.assertIn(("enum", "ExposureAuto", "Off"), _FakeCamera.configured)
            self.assertIn(("enum", "GainAuto", "Off"), _FakeCamera.configured)
            self.assertIn(("enum", "BalanceWhiteAuto", "Off"), _FakeCamera.configured)
            for name in (
                "GammaEnable",
                "SaturationEnable",
                "SharpnessEnable",
                "BlackLevelEnable",
            ):
                self.assertIn(("bool", name, False), _FakeCamera.configured)
            for statuses in backend.correction_status_by_serial.values():
                self.assertEqual(
                    statuses["SaturationEnable"],
                    "UNAVAILABLE_IN_MONO8_FEATURE_SET",
                )
                self.assertEqual(
                    {
                        statuses["GammaEnable"],
                        statuses["SharpnessEnable"],
                        statuses["BlackLevelEnable"],
                    },
                    {"OFF_VERIFIED"},
                )
            backend.close()
        _FakeCamera.inaccessible_bool_nodes.clear()


if __name__ == "__main__":
    unittest.main(verbosity=2)

# PatchCore offline model tools

운영 Vision Node는 위치별 정상 variation을 보정하는 PatchCore artifact v3만
허용합니다. 기존 `MB_construction.py`, `patchcore_AD.py`와 artifact v2는 재현·비교를
위해 보존하지만 v3 runtime에는 배포할 수 없습니다.

- `MB_construction_2.py`: memory bank, 공간 calibration, 후보 선택과 artifact v3 생성
- `patchcore_AD_2.py`: 고정된 artifact v3를 test set에서 최종 평가
- `patchcore_v3_common.py`: full-frame 전처리, native patch map, 공간 점수 및 v3 계약

## Dataset 계약

모든 PNG는 카메라 canonical 해상도의 8-bit 1-channel full-frame Mono8이어야 합니다.
각 logical split/class 아래에는 네 view 폴더가 있고, 한 split 안에서는 네 view의 제품
파일명 집합이 정확히 같아야 합니다.

```text
data_set/
├── training_set/{CAM_A_1,CAM_A_2,CAM_A_3,CAM_B_1}/*.png
├── calibration_set_A/{VIEW}/*.png
├── calibration_set_B/{VIEW}/*.png
├── validation_set/
│   ├── normal/{VIEW}/*.png
│   └── anomaly/{VIEW}/*.png
└── test_set/
    ├── normal/{VIEW}/*.png
    └── anomaly/{VIEW}/*.png
```

- `training_set`: 정상 sample로 view별 memory bank 생성
- `calibration_set_A`: 정상 native patch map의 위치별 center/scale 계산
- `calibration_set_B`: 정상 제품 4-view OR FPR 1% threshold 계산
- `validation_set`: FPR 1% 이하에서 product recall 최대 후보 선택
- `test_set`: artifact를 변경하지 않고 정확도와 추론 시간 최종 측정

같은 제품·연속 촬영 sequence·생산 lot가 서로 다른 split에 섞이지 않게 분리합니다.
Calibration B는 최소 100제품이 필요하고 1% tail 안정성을 위해 1,000제품 이상을
권장합니다.

## 공간 점수 정책

판정은 bilinear 확대 전 native nearest-memory patch distance map에서 수행합니다.
Calibration A에서 다음 후보를 만들고 네 view에 공통인 한 정책을 선택합니다.

- epsilon z-score: epsilon ratio `0.001, 0.01, 0.1`
- std floor: floor ratio `0.05, 0.1, 0.2`
- variance shrinkage: lambda `0.05, 0.1, 0.25, 0.5`
- median/scaled MAD: epsilon ratio `0.001, 0.01, 0.1`
- map percentile: `99.0, 99.5, 99.9, 100.0`
- top-k average: 절대 patch 수 `1, 3, 5, 10`

기존 reweighted PatchCore image score와 위치 정규화 없는 aggregation도 baseline으로
기록하지만 자동 선택 대상은 아닙니다. 후보 선택 순서는 다음과 같습니다.

1. validation 4-view OR product FPR ≤ 1%
2. product anomaly recall 최대
3. product FPR 최소
4. validation postprocess p95 최소
5. 더 단순한 normalization 우선

`MB_construction_2.py`의 `parameters_by_view`에서 backbone, feature layer, coreset ratio,
입력 해상도, construction batch/chunk 크기와 seed를 view별로 독립 설정할 수 있습니다.
그 결과 native patch grid가 view마다 달라도 허용됩니다. 절대 top-k 후보는 모든 view에서
유효해야 하므로 각 view의 patch 수보다 작거나 같아야 합니다. View score `S_v`와
calibration B threshold `T_v`에 대해 `S_v > T_v`일 때만 NG입니다.
보고 score는 `S_v / T_v`이며 기존 customized margin은 사용하지 않습니다. Station A는
세 view OR, Station B는 한 view, 최종 제품은 두 station OR로 판정합니다.

## Artifact v3

```text
artifact/
├── manifest.json
├── CAM_A_1/{model.pt,spatial_calibration.pt,calibration.json}
├── CAM_A_2/{model.pt,spatial_calibration.pt,calibration.json}
├── CAM_A_3/{model.pt,spatial_calibration.pt,calibration.json}
└── CAM_B_1/{model.pt,spatial_calibration.pt,calibration.json}
```

Manifest에는 full-frame 전처리 계약, 선택 후보와 모든 validation 결과, dataset digest,
view별 threshold, patch grid, memory-bank shape와 병렬 benchmark를 저장합니다. 실제
center/denominator tensor는 `spatial_calibration.pt`에 저장되며 artifact 전체 directory
SHA-256에 포함됩니다. Runtime은 v2와 불완전한 v3를 fail-closed로 거부합니다.

## 실행

```bash
cd offline_model_tools
/usr/bin/python3 MB_construction_2.py \
  --artifact-name MB_v3_resol_180 \
  --dataset-root /absolute/path/to/data_set \
  --artifact-root /absolute/path/to/artifact

/usr/bin/python3 patchcore_AD_2.py \
  --artifact-name MB_v3_resol_180 \
  --dataset-root /absolute/path/to/data_set \
  --artifact-root /absolute/path/to/artifact \
  --result-root /absolute/path/to/result
```

Test 결과에는 product confusion/accuracy, view score 분포, FP/FN normalized heat map과
Station A/B별 model-pipeline 및 full-frame end-to-end mean/median/p95/p99가 포함됩니다.
Heat map만 bilinear 확대하며 확대와 시각화는 판정에 영향을 주지 않습니다.

# PatchCore offline model tools

`MB_construction.py`는 정상 training/validation 데이터로 운영용 4-view memory-bank
artifact를 만드는 유일한 도구입니다. `patchcore_AD.py`는 완성된 artifact를 읽어
normal/anomaly test dataset의 성능만 평가하며 memory bank나 threshold를 변경하지
않습니다. 두 스크립트는 같은 폴더의 `ad_common.py`를 사용합니다.

## View별 설정

`MB_construction.py`의 `MB_CONFIGS[artifact_name]["parameters_by_view"]`에서
`CAM_A_1`, `CAM_A_2`, `CAM_A_3`, `CAM_B_1`을 각각 수정합니다. 각 view는
backbone, feature layers, coreset ratio, reweighting k, threshold percentile,
margin, 입력 해상도, batch/chunk 크기와 seed를 독립적으로 가집니다. 생성된 v2
manifest도 같은 `parameters_by_view` 구조를 사용하므로 view별 memory-bank shape가
달라도 정상입니다.

## 실행

```bash
cd offline_model_tools
/usr/bin/python3 MB_construction.py \
  --artifact-name MB_resol_180_default \
  --dataset-root /absolute/path/to/data_set \
  --artifact-root /absolute/path/to/artifact

/usr/bin/python3 patchcore_AD.py \
  --artifact-name MB_resol_180_default \
  --dataset-root /absolute/path/to/data_set \
  --artifact-root /absolute/path/to/artifact \
  --result-root /absolute/path/to/result
```

Dataset split은 `training_set`, `val_set`, `test_set_normal`,
`test_set_anomaly`이고, 각 split 안에 네 view 폴더와 view 간 동일한 제품 PNG
파일명 집합이 있어야 합니다. 운영 계약은 Mono8 또는 세 RGB channel 값이 동일한
8-bit PNG만 허용합니다.

평가 결과의 각 view 폴더에는 `normalized_score_distribution.png`가 생성됩니다.
`false-positive`와 `false-negative` 제품 폴더에는 view별 model-input PNG, 실제
PatchCore patch distance를 겹친 `*_heatmap.png`, 화면과 파일명에 표시된 normalized
score, 그리고 전체 수치를 담은 `scores.json`이 함께 저장됩니다.

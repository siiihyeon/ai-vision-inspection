# PatchCore v3 artifact 운영 배포 매뉴얼

이 문서는 MB_construction_2.py가 생성하고 patchcore_AD_2.py의 최종 시험을 통과한
PatchCore v3 artifact를 실제 inspection_vision 모델에 주입하는 절차를 설명합니다.
운영 runtime은 개별 model.pt가 아니라 **artifact 디렉터리 전체**를 하나의 불변
bundle로 검증하고 로드합니다.

## 1. 배포 원칙

- 실제 장비에는 profile:=hardware를 사용합니다.
- artifact마다 중복되지 않는 새 이름을 사용하며 기존 운영 디렉터리를 덮어쓰지
  않습니다.
- artifact를 배치한 뒤에는 내부 파일을 수정하거나 추가하지 않습니다.
- 일반 sha256sum model.pt가 아니라 프로젝트의 **통합 directory SHA-256**을
  사용합니다. 이 값은 정렬된 상대경로와 모든 파일 내용을 함께 계산합니다.
- vision.model.path, vision.model.version, vision.model.sha256은 한 세트로 변경합니다.
- 설정 파일은 src 아래 원본을 수정합니다. build 또는 install 아래 복사본을 직접
  수정하지 않습니다.
- 배포 전 최종 test 결과와 기존 운영 artifact의 path/version/SHA를 기록합니다.

## 2. Runtime이 요구하는 파일 구조

Artifact에는 아래 13개 파일만 있어야 합니다. 추가 메모, 이미지, 로그 또는 숨김
파일도 허용되지 않으며 symlink도 허용되지 않습니다.

    <ARTIFACT_NAME>/
    ├── manifest.json
    ├── CAM_A_1/{model.pt,calibration.json,spatial_calibration.pt}
    ├── CAM_A_2/{model.pt,calibration.json,spatial_calibration.pt}
    ├── CAM_A_3/{model.pt,calibration.json,spatial_calibration.pt}
    └── CAM_B_1/{model.pt,calibration.json,spatial_calibration.pt}

manifest.json의 format_version은 3, algorithm은 patchcore여야 합니다.
model_version은 배포할 vision.model.version과 정확히 같아야 합니다. 새 ratio-based
Top-k artifact는 aggregation에 top_k_percent, rounding=ceil,
minimum_patch_count=1을 기록합니다. 기존 v2 또는 절대 Top-k artifact는 runtime이
거부합니다.

## 3. 예시 배포 변수 준비

아래 예시는 repository가 ~/ai-vision-inspection-main에 있고 artifact 이름이
MB_prod_2026_09_01이라고 가정합니다. 실제 경로와 이름으로 바꿉니다.

    export DEPLOY_REPOSITORY="$HOME/ai-vision-inspection-main"
    export DEPLOY_ARTIFACT_NAME="MB_prod_2026_09_01"
    export DEPLOY_SOURCE_DIR="$DEPLOY_REPOSITORY/offline_model_tools/artifact/$DEPLOY_ARTIFACT_NAME"
    export DEPLOY_MODEL_ROOT="/opt/inspection/models"
    export DEPLOY_FINAL_DIR="$DEPLOY_MODEL_ROOT/$DEPLOY_ARTIFACT_NAME"

각 명령 전에 값이 의도한 절대경로인지 확인합니다.

    printf 'repository=%s\nartifact=%s\nsource=%s\nfinal=%s\n' \
      "$DEPLOY_REPOSITORY" "$DEPLOY_ARTIFACT_NAME" \
      "$DEPLOY_SOURCE_DIR" "$DEPLOY_FINAL_DIR"
    test -d "$DEPLOY_REPOSITORY/.git"
    test -d "$DEPLOY_SOURCE_DIR"

## 4. Step 1 — Artifact 생성 완료 여부 확인

Artifact를 아직 생성하지 않았다면 offline_model_tools에서 생성합니다.

    cd "$DEPLOY_REPOSITORY/offline_model_tools"
    /usr/bin/python3 MB_construction_2.py \
      --artifact-name "$DEPLOY_ARTIFACT_NAME" \
      --dataset-root /absolute/path/to/data_set \
      --artifact-root "$DEPLOY_REPOSITORY/offline_model_tools/artifact"

생성 도중 normalization/aggregation 후보, calibration B threshold, validation
FPR/recall과 병렬 benchmark가 결정되어 artifact에 저장됩니다. 생성이 성공적으로
끝나기 전의 임시 디렉터리는 배포하지 않습니다.

## 5. Step 2 — 고정 test set 최종 평가

정책과 threshold를 더 이상 변경하지 않을 상태에서 test set을 한 번 평가합니다.

    cd "$DEPLOY_REPOSITORY/offline_model_tools"
    /usr/bin/python3 patchcore_AD_2.py \
      --artifact-name "$DEPLOY_ARTIFACT_NAME" \
      --dataset-root /absolute/path/to/data_set \
      --artifact-root "$DEPLOY_REPOSITORY/offline_model_tools/artifact" \
      --result-root /absolute/path/to/final_test_results

배포 승인 전에 최소한 다음을 확인합니다.

- metrics.json: confusion matrix, F1, recall, precision, FPR
- inference_time.json: Station A/B의 model-forward-only,
  entire-model-pipeline, end-to-end 통계
- run_information.json: artifact SHA, 선택 정책, library version, dataset provenance
- TP/TN/FP/FN 폴더: 모든 제품의 원본, model input, heat map, overlay, score
- 4-view OR recall과 FPR이 승인 기준을 만족하는지

Test 결과를 보고 정책이나 threshold를 변경했다면 기존 test set은 더 이상 최종
holdout이 아닙니다. 새 artifact와 새로운 독립 test set으로 다시 평가합니다.

## 6. Step 3 — Source artifact 정적 점검

파일 목록을 확인합니다.

    find "$DEPLOY_SOURCE_DIR" -type f -printf '%P\n' | sort

Symlink가 하나도 없어야 합니다.

    test -z "$(find "$DEPLOY_SOURCE_DIR" -type l -print -quit)"

Manifest identity와 선택 정책을 확인합니다.

    /usr/bin/python3 -c '
    import json, os
    from pathlib import Path
    p = Path(os.environ["DEPLOY_SOURCE_DIR"]) / "manifest.json"
    m = json.loads(p.read_text(encoding="utf-8"))
    print("format_version:", m["format_version"])
    print("algorithm:", m["algorithm"])
    print("artifact_name:", m["artifact_name"])
    print("model_version:", m["model_version"])
    print("selected_candidate_id:", m["candidate_selection"]["selected_candidate_id"])
    print("normalization:", m["spatial_scoring_policy"]["normalization"])
    print("aggregation:", m["spatial_scoring_policy"]["aggregation"])
    print("decision:", m["spatial_scoring_policy"]["decision"])
    print("thresholds:", m["thresholds"])
    '

다음 세 값은 동일하게 유지하는 것을 권장합니다.

    배포 디렉터리 basename
    manifest.json의 artifact_name/model_version
    vision.model.version

## 7. Step 4 — Source directory SHA-256 계산

일반 파일 SHA가 아닌 프로젝트 함수를 사용합니다.

    cd "$DEPLOY_REPOSITORY/offline_model_tools"
    export DEPLOY_SOURCE_SHA="$(
      /usr/bin/python3 -c '
    import os
    from pathlib import Path
    from ad_common import artifact_directory_sha256
    print(artifact_directory_sha256(Path(os.environ["DEPLOY_SOURCE_DIR"])))
    '
    )"
    printf 'source artifact SHA-256=%s\n' "$DEPLOY_SOURCE_SHA"
    printf '%s' "$DEPLOY_SOURCE_SHA" | grep -Eq '^[0-9a-f]{64}$'

patchcore_AD_2.py 시작 화면과 run_information.json의 SHA도 같아야 합니다.

## 8. Step 5 — 실행 중인 시스템 안전 종료

운영 중에 artifact나 YAML을 교체하지 않습니다.

1. 신규 제품 투입을 중단합니다.
2. 추적 중인 제품이 모두 배출되었는지 확인합니다.
3. 필요하면 승인된 operator 절차로 pause/line-clear를 수행합니다.
4. launch 터미널에서 Ctrl+C를 눌러 네 노드의 안전 종료를 기다립니다.
5. ros2 node list에서 이전 노드가 남아 있지 않은지 확인합니다.

Artifact 교체는 컨베이어와 노드가 정지한 maintenance window에서만 수행합니다.

## 9. Step 6 — /opt/inspection/models에 versioned bundle 배치

최초 한 번 모델 루트를 준비합니다. 실제 서비스 계정이 따로 있다면 그 계정을
소유자로 지정합니다.

    sudo install -d -m 0755 "$DEPLOY_MODEL_ROOT"

동일 이름이 이미 존재하면 덮어쓰지 말고 새 artifact 이름을 사용합니다.

    test ! -e "$DEPLOY_FINAL_DIR"

같은 filesystem 안의 staging 디렉터리로 복사한 다음 rename합니다.

    export DEPLOY_STAGE_DIR="$DEPLOY_MODEL_ROOT/.$DEPLOY_ARTIFACT_NAME.staging"
    test ! -e "$DEPLOY_STAGE_DIR"
    sudo install -d -m 0755 "$DEPLOY_STAGE_DIR"
    sudo cp -a "$DEPLOY_SOURCE_DIR/." "$DEPLOY_STAGE_DIR/"
    sudo chmod -R a+rX "$DEPLOY_STAGE_DIR"
    sync

Staging 복사본의 SHA를 다시 계산합니다.

    cd "$DEPLOY_REPOSITORY/offline_model_tools"
    export DEPLOY_STAGE_SHA="$(
      /usr/bin/python3 -c '
    import os
    from pathlib import Path
    from ad_common import artifact_directory_sha256
    print(artifact_directory_sha256(Path(os.environ["DEPLOY_STAGE_DIR"])))
    '
    )"
    printf 'source=%s\nstaging=%s\n' "$DEPLOY_SOURCE_SHA" "$DEPLOY_STAGE_SHA"
    test "$DEPLOY_SOURCE_SHA" = "$DEPLOY_STAGE_SHA"

SHA가 일치할 때만 최종 이름으로 원자적으로 rename합니다.

    sudo mv "$DEPLOY_STAGE_DIR" "$DEPLOY_FINAL_DIR"
    test -d "$DEPLOY_FINAL_DIR"

배포 후 artifact 안에 파일을 추가하거나 내용 변경을 하면 SHA가 달라집니다. 로그와
시험 결과는 artifact 디렉터리 밖에 보관합니다.

## 10. Step 7 — Hardware model YAML 갱신

다음 source 파일을 수정합니다.

    ros2_ws/src/basic_packages/inspection_bringup/config/vision_model.hardware.yaml

세 identity 값을 함께 바꿉니다.

    /inspection/vision_node:
      ros__parameters:
        vision.model.path: "/opt/inspection/models/MB_prod_2026_09_01"
        vision.model.version: "MB_prod_2026_09_01"
        vision.model.sha256: "<DEPLOY_SOURCE_SHA 64자리 소문자>"
        vision.model.runtime: "PYTORCH_PATCHCORE_ARTIFACT"
        vision.model.cuda_required: true
        vision.model.warmup_runs: 10
        vision.model.serialize_access: true
        vision.worker_count: 1

주의사항:

- vision.model.path는 존재하는 절대 디렉터리여야 합니다.
- vision.model.version은 manifest의 model_version과 정확히 같아야 합니다.
- SHA는 64자리 소문자 hexadecimal로 입력합니다.
- scoring, V-threshold, 입력 해상도와 feature layer를 ROS YAML에 중복 작성하지
  않습니다.
- worker/model-lock은 별도 생산 성능 시험 없이 artifact 배포와 동시에 변경하지 않는
  것을 권장합니다.

## 11. Step 8 — ROS 검증 및 빌드

Repository 루트에서 정적 검사와 Python test를 실행합니다.

    cd "$DEPLOY_REPOSITORY"
    /usr/bin/python3 ros2_ws/tools/verify_skeleton.py
    /usr/bin/python3 ros2_ws/tools/test_domain_contracts.py
    /usr/bin/python3 ros2_ws/tools/test_vision_algorithms.py
    MPLCONFIGDIR=/tmp/patchcore-v3-mpl \
      /usr/bin/python3 ros2_ws/tools/test_patchcore_v3_policy.py

그다음 workspace를 빌드하고 package test를 실행합니다.

    cd "$DEPLOY_REPOSITORY/ros2_ws"
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install --cmake-force-configure \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
    source install/setup.bash
    colcon test
    colcon test-result --verbose

error/failure가 있으면 배포를 진행하지 않습니다. 설치된 hardware YAML이 source와
같은지도 확인합니다.

    export DEPLOY_BRINGUP_PREFIX="$(ros2 pkg prefix inspection_bringup)"
    grep -n 'vision.model' \
      "$DEPLOY_BRINGUP_PREFIX/share/inspection_bringup/config/vision_model.hardware.yaml"

## 12. Step 9 — Hardware profile 실행

첫 번째 터미널에서 실행합니다.

    cd "$DEPLOY_REPOSITORY/ros2_ws"
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch inspection_bringup inspection_system.launch.py profile:=hardware

Launch 직후 두 번째 터미널에서 model parameter를 확인합니다.

    cd "$DEPLOY_REPOSITORY/ros2_ws"
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 param get /inspection/vision_node vision.model.path
    ros2 param get /inspection/vision_node vision.model.version
    ros2 param get /inspection/vision_node vision.model.sha256
    ros2 param get /inspection/vision_node vision.model.runtime

값이 틀리면 init을 실행하지 말고 launch를 종료한 뒤 source YAML/build/source 순서를
다시 확인합니다.

## 13. Step 10 — InitializeNode로 실제 모델 로드

모델은 launch 순간이 아니라 Master의 initialize 명령 과정에서 검증·GPU 로드·warmup
됩니다.

    cd "$DEPLOY_REPOSITORY/ros2_ws"
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ./tools/operator_command.sh init "deploy $DEPLOY_ARTIFACT_NAME"

Vision 초기화는 다음을 모두 통과해야 성공합니다.

1. 카메라/NIC/firmware 및 acquisition 설정 검증
2. Artifact directory SHA 일치
3. v3 manifest와 정확한 파일 집합 검증
4. Serial-to-view 및 Station A/B view 순서 검증
5. View별 model/memory bank/spatial calibration 로드
6. 선택 normalization/aggregation/threshold 계약 검증
7. CUDA device 0 로드
8. Warmup 및 저장된 병렬도의 현재 VRAM reserve 검증

성공 로그 reason은 다음과 같습니다.

    MVS Action1 and four-view PatchCore artifact initialized

Heartbeat에서 Vision health가 READY(1)인지 확인합니다.

    ros2 topic echo --once /inspection/vision/heartbeat

INIT_BLOCKED, DEGRADED 또는 FAULT이면 start를 실행하지 않습니다. Launch 터미널의
Vision hardware initialization failed 원인을 해결한 뒤 새 init 요청을 보냅니다.

## 14. Step 11 — 제한된 smoke test

전체 생산 운전 전에 라인이 비어 있는 상태에서 승인된 소량의 정상/불량 제품으로
시험합니다.

- 네 카메라 serial/view 대응이 맞는지
- Station A 3-view 및 Station B 1-view가 모두 추론되는지
- 결과의 model_version과 model_sha256이 새 artifact인지
- View별 score와 최종 4-view OR 판정이 예상과 맞는지
- CUDA OOM이나 artifact/library warning이 없는지
- model-forward, queue-to-terminal, end-to-end 지연이 station timeout 안에 있는지
- 정상 FPR과 anomaly recall이 offline test와 크게 어긋나지 않는지

Smoke test를 통과한 뒤에만 승인된 운전 절차로 production run을 시작합니다.

## 15. Rollback 절차

새 artifact 초기화나 smoke test가 실패하면 기존 artifact를 삭제하거나 덮어쓰지 않고
identity 세 값을 이전 값으로 되돌립니다.

1. 신규 제품 투입을 중단하고 라인을 안전 상태로 만듭니다.
2. Launch를 Ctrl+C로 종료합니다.
3. vision_model.hardware.yaml의 path/version/SHA를 이전 승인값으로 복원합니다.
4. ROS workspace를 다시 빌드하고 source install/setup.bash를 실행합니다.
5. Hardware profile을 다시 launch합니다.
6. operator_command.sh init을 실행합니다.
7. Heartbeat READY와 이전 model version/SHA를 확인합니다.

Rollback이 검증될 때까지 실패한 새 artifact도 삭제하지 말고 장애 분석 대상으로
보존합니다.

## 16. 자주 발생하는 실패와 조치

| 오류 | 원인 | 조치 |
|---|---|---|
| model artifact SHA-256 differs | 복사 오류, 배포 후 변경, 잘못된 YAML SHA | 배포 디렉터리 SHA를 다시 계산하고 source/staging SHA 비교 |
| artifact file set differs | 파일 누락 또는 artifact 안에 추가 파일 존재 | 허용된 13개 파일만 새 이름의 디렉터리에 다시 배포 |
| artifact symlink is not allowed | 복사본에 symlink 포함 | 실제 파일로 구성된 artifact를 다시 배포 |
| unsupported format_version | v2 또는 구형 v3 artifact | 최신 MB_construction_2.py로 새 artifact 생성 |
| artifact model_version differs | YAML version과 manifest 불일치 | manifest model_version을 YAML에 정확히 입력 |
| camera serial/view mapping differs | Artifact와 hardware camera map 불일치 | 우회하지 말고 artifact와 실제 serial mapping 재검증 |
| CUDA device 0 is unavailable | Driver/CUDA/GPU 문제 | nvidia-smi, PyTorch CUDA 인식과 서비스 계정 권한 확인 |
| parallel_count is unsafe | 현재 free VRAM이 reserve 조건 미달 | 다른 GPU 프로세스를 종료하고 재시도; 필요하면 artifact 재생성 |
| library compatibility warning | 생성 환경과 운영 torch/torchvision 차이 | 동일 환경을 우선 사용하고 smoke test 결과를 승인 기록에 남김 |
| Installed YAML이 이전 값 | source YAML 수정 후 build/source 누락 | bringup rebuild 후 새 install/setup.bash source |

## 17. 최종 배포 체크리스트

- [ ] 최종 artifact가 독립 test set 평가를 통과했다.
- [ ] Artifact 이름/version이 고유하고 manifest와 일치한다.
- [ ] 허용된 13개 파일만 있고 symlink가 없다.
- [ ] Source와 /opt staging SHA가 같다.
- [ ] vision.model.path/version/sha256 세 값을 함께 갱신했다.
- [ ] 기존 승인 artifact identity를 rollback용으로 기록했다.
- [ ] 정적 검사, Python test, ROS build/test가 통과했다.
- [ ] 설치된 hardware YAML 값이 source와 같다.
- [ ] Hardware launch 후 ROS parameter 값을 확인했다.
- [ ] init 결과와 Vision heartbeat READY를 확인했다.
- [ ] 제한된 smoke test에서 모델 identity, 판정, 지연, GPU 상태를 확인했다.
- [ ] 생산 승인 기록에 artifact SHA와 test result 경로를 남겼다.

# 검증 도구

- `verify_skeleton.py`: ROS 설치 없이 파일, IDL field, version, 금지된 LED/hardware trigger 계약과 Python AST를 검사합니다.
- `test_domain_contracts.py`: stdlib만으로 Master 결합/FIFO, Vision queue, idempotency, SQLite spool/repository를 시험합니다.

두 검사는 Jazzy `colcon build`를 대체하지 않습니다. ZIP 생성 전과 모든 PR에서 함께 실행합니다.
## Vision artifact와 fake-SDK 검증

```bash
python3 tools/test_vision_algorithms.py
python3 tools/inspect_patchcore_artifact.py /absolute/artifact/path \
  --version MB_resol_180_default
```

두 번째 명령은 CUDA를 사용하지 않고 v2 manifest/file/calibration 계약을
검증한 뒤 hardware YAML에 넣을 통합 directory SHA-256을 출력합니다.

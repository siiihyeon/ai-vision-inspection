# 검증 도구

- `verify_skeleton.py`: ROS 설치 없이 파일, IDL field, version, 금지된 LED/hardware trigger 계약과 Python AST를 검사합니다.
- `test_domain_contracts.py`: stdlib만으로 Master 결합/FIFO, Vision queue, idempotency, SQLite spool/repository를 시험합니다.

두 검사는 Jazzy `colcon build`를 대체하지 않습니다. ZIP 생성 전과 모든 PR에서 함께 실행합니다.

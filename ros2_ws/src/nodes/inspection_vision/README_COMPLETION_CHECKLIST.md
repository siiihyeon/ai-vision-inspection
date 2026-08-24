# Vision Node 완료·인수 체크리스트

## 코드로 완료된 항목

- [x] Serial/IP/view 고정 매핑과 Station A 3-view, B 1-view 순서 검증
- [x] GigE Action1 key/mask 분리와 ACK IP/status fail-closed 검증
- [x] 2448×2048 Mono8 전체 ROI, acquisition 250 ms, skew 50 ms
- [x] 카메라별 exposure/gain 적용 및 모든 자동·영상 보정 OFF
- [x] packet delay 5000, frame packet loss 0 계약
- [x] V threshold, threshold, margin을 artifact에만 보관
- [x] Mono8 foreground/connected-component/crop/padding 전처리
- [x] ResNet34/EfficientNet PatchCore memory bank loader와 CUDA 추론
- [x] View NG 결합과 normalized station max score
- [x] NG/전처리 실패에 한정한 crop 진단 저장
- [x] Artifact v2 파일 집합, serial/view, calibration, version, 통합 SHA 검증
- [x] A/B timeout 3000/1500 ms와 10,000표본 자동 tuning guard
- [x] 전처리 실패 분류, 파일-read-only retry, CUDA OOM 재초기화 경로
- [x] durable-before-terminal, session-local cache cleanup
- [x] MVS fake-SDK와 전처리/artifact 회귀 시험
- [x] feature branch에서도 정적 검사와 ROS Jazzy build/test를 수행하는 CI

## 실제 제품 실행 전 차단 사항

- [ ] 최종 4-view format v2 artifact bundle을 생성한다. 현재 제공된 v1
  artifact는 B view가 없어 운영 loader가 의도적으로 거부한다.
- [ ] 최종 bundle의 `preprocessing_by_view`에는 네 view 모두 초기
  `v_threshold=40`을 넣는다. 판정 threshold와 margin 초기값 0.02도 artifact
  생성 결과에만 두고 ROS YAML이나 runtime 코드에는 복제하지 않는다.
- [ ] 최종 bundle의 통합 SHA-256을
  `vision_model.hardware.yaml`의 `vision.model.sha256`에 입력한다.
- [ ] 실제 네 카메라에서 model/serial/IP/firmware가 검증되는지 확인한다.
  실제 firmware 값은 현재 미확정이며, 기대값이 비어 있으면 네 대의 동일성만
  검증한다.
- [ ] 카메라가 연결된 상태에서 각 보정 GenICam node가 writable이고 OFF인지
  MVS UI와 초기화 로그 양쪽에서 확인한다.
- [ ] 케이블 disconnect/reconnect와 5초 지속 단절 PAUSE 인수 시험을 한다.
- [ ] 컨베이어 통합 시 A→Sensor3 약 4초, B→Sensor3 최소 2초 조건과
  Sensor3 forced-NG/late-result 경로를 시험한다.

## 생산 승인 전 성능 시험

- [ ] A 3대와 B 1대 단독/동시 trigger에서 packet loss가 계속 0인지 확인한다.
- [ ] Host arrival skew가 50 ms 이내인지 장시간 측정한다.
- [ ] A/B 정상 추론 각각 10,000표본으로 enqueue→terminal p99.9를 구하고
  1.2 안전계수 후보를 검토한다. 그 전에는 3000/1500 ms를 유지한다.
- [ ] queue 16, worker 1, model lock true에서 backlog·GPU·VRAM·온도를 측정한다.
- [ ] Disk 90% warning, 95% stop, 최근 10,000장 보존을 검증한다.
- [ ] 정상 종료, 강제 종료, Vision/Log 재시작에서 멱등성과 spool replay를 검증한다.

코드 구현 완료와 생산 승인은 다릅니다. 위 첫 번째 차단 묶음이 해결되어야
hardware profile이 READY가 될 수 있고, 실제 장비·공정 시험을 통과해야 생산
승인을 선언할 수 있습니다.

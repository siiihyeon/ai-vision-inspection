# Vision Node 완료·인수 체크리스트

## 코드로 완료된 항목

- [x] Serial/IP/view 고정 매핑과 Station A 3-view, B 1-view 순서 검증
- [x] GigE Action1 key/mask 분리와 ACK IP/status fail-closed 검증
- [x] 2448×2048 Mono8 전체 ROI, acquisition 250 ms, skew 50 ms
- [x] 카메라별 exposure/gain 적용 및 모든 자동·영상 보정 OFF
- [x] packet delay 5000, frame packet loss 0 계약
- [x] Full-frame 전처리 버전, V threshold, 공간 통계와 threshold를 artifact에만 보관
- [x] Mono8 foreground/connected-component/crop/padding 전처리
- [x] ResNet34/EfficientNet PatchCore memory bank loader와 CUDA 추론
- [x] Native patch 위치 정규화, percentile/top-k view score와 strict threshold 판정
- [x] View NG 결합과 threshold-ratio station max score
- [x] NG/전처리 실패에 한정한 crop 진단 저장
- [x] Artifact v3 파일 집합, serial/view, 공간 calibration, version, 통합 SHA 검증
- [x] Artifact v2 명시적 거부와 offline/runtime 공간 점수 golden test
- [x] A/B timeout 3000/1500 ms와 10,000표본 자동 tuning guard
- [x] 전처리 실패 분류, 파일-read-only retry, CUDA OOM 재초기화 경로
- [x] durable-before-terminal, session-local cache cleanup
- [x] MVS fake-SDK와 전처리/artifact 회귀 시험
- [x] feature branch에서도 정적 검사와 ROS Jazzy build/test를 수행하는 CI

## 실제 제품 실행 전 차단 사항

- [ ] 합의된 full-frame training/calibration A/calibration B/validation/test split으로
  최종 4-view format v3 artifact를 생성한다. Calibration B는 최소 100, 권장 1,000
  정상 제품이며 validation 4-view OR FPR 1% 이하 조건을 통과해야 한다.
- [ ] 최종 v3 bundle을 `/opt/inspection/models`에 배포하고 artifact 이름과 통합
  SHA-256을 `vision_model.hardware.yaml`에 입력한다. 기존 배포 v2 bundle과 SHA는
  v3 runtime에서 의도적으로 거부되므로 재사용할 수 없다.
- [ ] `patchcore_AD_2.py`로 고정 test set의 최종 정확도와 Station A/B별 model
  pipeline/end-to-end mean·median·p95·p99를 기록한다.
- [x] 2026-08-25 MVS SDK 열거에서 네 카메라의 model/serial/IP가 설정과
  일치하고 firmware가 모두 `V4.0.43 250414 1530132`임을 확인했다.
- [x] 2026-08-25 실카메라 초기화에서 exposure/gain/white balance 자동 기능을
  OFF로 적용했다. Gamma·sharpness·black level은 네 대 모두 read-back OFF,
  saturation은 Mono8 feature set에서 비활성 node임을 기록했다.
- [ ] 케이블 disconnect/reconnect와 5초 지속 단절 PAUSE 인수 시험을 한다.
- [ ] 컨베이어 통합 시 A→Sensor3 약 4초, B→Sensor3 최소 2초 조건과
  Sensor3 forced-NG/late-result 경로를 시험한다.

## 생산 승인 전 성능 시험

- [x] 2026-08-25 1회 smoke test에서 A 3대와 B 1대 Action ACK, 2448×2048
  Mono8, packet loss/resend 0을 확인했다. A host-arrival skew는 12.877 ms였다.
- [ ] A 3대와 B 1대 단독/동시 trigger에서 packet loss가 계속 0인지 확인한다.
- [ ] Host arrival skew가 50 ms 이내인지 장시간 측정한다.
- [ ] A/B 정상 추론 각각 10,000표본으로 enqueue→terminal p99.9를 구하고
  1.2 안전계수 후보를 검토한다. 그 전에는 3000/1500 ms를 유지한다.
- [ ] queue 16, worker 1, model lock true에서 backlog·GPU·VRAM·온도를 측정한다.
- [ ] Disk 90% warning, 95% stop, 최근 10,000장 보존을 검증한다.
- [ ] 정상 종료, 강제 종료, Vision/Log 재시작에서 멱등성과 spool replay를 검증한다.

코드 구현과 artifact 배포 완료는 생산 승인을 뜻하지 않습니다. 남은 실제
장비·공정 인수시험을 모두 통과한 뒤에만 생산 승인을 선언할 수 있습니다.

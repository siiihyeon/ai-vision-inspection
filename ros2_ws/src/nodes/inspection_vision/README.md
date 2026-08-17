# inspection_vision

Vision은 camera capture, RGB PNG 파일, station-level FrameBatch, bounded FIFO, shared model worker와 station 결과를 소유합니다. 제품 최종 판정은 소유하지 않습니다.

## 골격에 구현된 경계

- station A/B 각각의 async lock: 같은 station 직렬, 서로 다른 camera set이면 A/B 동시 가능
- `CaptureBackend` Protocol과 fail-closed placeholder
- 같은 `capture_id`로 필수 camera 전체 최대 2 attempts
- `CaptureBatch` identity, 필수 camera, 절대경로, RGB8 PNG, digest/size 검증
- host arrival monotonic max-min `frame_arrival_skew_us`
- queue full 시 saved batch 유지, Action `ENQUEUE_BLOCKED`, 복구 후 같은 job enqueue
- path-only bounded FIFO와 Sensor3 lock 제거
- shared model `WorkerPool`, model lock 기본 활성, file read 1 retry, inference 1 retry

## 반드시 결정할 Camera/MVS 값

| 항목 | 정확히 필요한 정보 |
|---|---|
| 장치 | MV-CS050-10GC 각 serial, station/role, NIC와 IP topology |
| SDK | Ubuntu 24용 MVS SDK 정확한 version, Python binding/API, camera firmware |
| Action | 장치별 Action1 지원, `TriggerSource=Action1`, device key, group key/mask, scheduled action 사용 여부 |
| Network | NIC MTU/jumbo frame, packet size/delay, bandwidth reserve, firewall, reconnect |
| Pixel | camera Bayer format, exposure/gain, white balance, demosaic algorithm, 출력 2448×2048 RGB8 PNG 확인 |
| Timing | acquisition timeout, callback의 host arrival 기록 지점, camera raw timestamp unit/wrap/domain |
| PTP | IEEE1588 지원·동기화 시험 결과와 camera timestamp 사용 가능 여부 |
| Skew | `frame_arrival_skew_limit_us` production 값과 시험 분포 |
| Recovery | camera별 장애 판정, 해당 station 시험촬영 성공 조건, late frame 보존 경로 |

## 반드시 결정할 Queue/Model/파일 값

| 항목 | 정확히 필요한 정보 |
|---|---|
| Queue | capacity, enqueue→결과 총 timeout ms, 메모리/디스크 포화 기준 |
| Worker | production 수, 공유 model thread safety, lock 유지/해제 근거, CPU/GPU affinity |
| Model | artifact path/version/SHA-256, runtime, device, warmup, 입력 shape/batch |
| 전처리 | OpenCV load BGR→RGB, resize/crop, scale/normalize, camera order |
| 판정 | 수학식, score 범위, camera 결과 결합, station threshold, 불확실/오류 처리 |
| 파일 | `data_root`, attempt/frame naming, temp suffix+fsync+atomic rename, permission |
| 보존 | 성공/NG/실패/late image 보존 기간, disk warning/stop, Log 삭제 요청 handshake |

`StationResult.score`는 유한한 `float32`만 허용합니다. 계산 결과가 `NaN`·`Inf`이면
`StationResult`를 발행하지 말고 `StationInferenceFailed`로 보고해야 합니다.
비유한 score가 전송되면 Master는 결과 계약 위반으로 기록하고 해당 제품을
`FORCED_NG` 처리합니다. score의 수학적 의미·정상 범위·threshold는 별도
모델 검증으로 확정합니다.

## 필수 구현 순서

1. fake `CaptureBackend` 통합 test
2. MVS device enumeration/config/ARM과 GIGE Action adapter
3. atomic RGB PNG writer + digest/readback
4. model loader/worker callbacks → `StationResult`/`StationInferenceFailed`
5. restart recovery: 저장 완료 batch를 재사용하지 않고 제품 잔류 시 재촬영, 이탈 시 FORCED_NG 보고
6. station camera recovery test capture

Action 정상 결과는 `error_code=0`, `reason=""`; warning은 `LogEvent`입니다.

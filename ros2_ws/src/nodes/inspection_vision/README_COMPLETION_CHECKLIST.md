# Vision Node 완성 결정표와 파라미터 위치

이 문서는 Vision Node를 실제 생산 운전에 투입하기 전에 확정하거나 시험해야 할 항목의 단일 기준입니다. 현재 승인 정책, 아직 필요한 정책, 구체 값, 실험 결과, 설정 위치를 모두 구분합니다. 과거 문서와 충돌하면 이 문서와 실제 hardware YAML을 우선합니다.

## 상태 표기

| 상태 | 의미 |
|---|---|
| 확정 | 정책과 값이 승인됐으며 현재 코드 또는 YAML에 반영됨 |
| 초기값 | 첫 시험을 위한 값일 뿐 생산 승인값은 아님 |
| 자동 조회 | 실행 시 SDK 또는 파일에서 읽어 검증함 |
| 결정 필요 | 구현을 끝내기 전에 사용자·모델 담당자와 논리 정책을 확정해야 함 |
| 실험 필요 | 실장비 데이터로 생산값을 정해야 함 |
| 해당 없음 | 현재 Mono8·host monotonic 정책에서는 필요하지 않음 |

## 완성의 의미

다음 두 단계는 서로 다릅니다.

1. 구현 완성: 실제 TorchScript 전처리·출력 decoder를 넣고 hardware profile이 READY가 되도록 모든 필수값을 입력한 상태
2. 생산 승인: Ubuntu PC, 네 카메라, GigE 스위치, 실제 모델과 공정 속도로 장시간 시험하여 packet loss, skew, timeout, GPU, 저장장치 기준을 통과한 상태

결정값만 입력하면 구현 완성은 가능하지만, 실장비 시험 없이 생산 승인을 선언할 수는 없습니다.

Mega 연결 방식과 firmware protocol은 Vision Node 내부 구현을 막는 입력은 아닙니다. 다만 Sensor3 deadline, 재시작 잔류 판정과 전체 라인 안전 운전을 검증하려면 Control/Master 통합 전에 반드시 확정해야 합니다.

## 이미 확정된 핵심 정책

| 영역 | 확정 내용 | 변경 위치 |
|---|---|---|
| 역할 | Vision은 station 결과만 만들고 제품 최종 판정은 Master가 소유 | 정책 변경은 inspection_vision, inspection_master, inspection_interfaces 동시 검토 |
| Station A | 3 view를 한 batch로 한 번 forward하며 하나라도 NG이면 terminal NG | 모델 결합 로직은 inspection_vision/model_backend.py |
| Station B | 1 view를 한 batch로 forward | inspection_vision/model_backend.py |
| A NG 취소 | B 미촬영·저장 후 enqueue 전·queue 대기·pre-forward는 취소, active forward는 완료 후 결과 폐기 | inspection_vision/inference_queue.py와 vision_node.py |
| Sensor3 | 결과 미완료 시 Master가 FORCED_NG로 잠그며 이후 Vision 결과는 공정에 미적용 | inspection_master |
| Queue 순서 | job은 fifo_sequence 순서로 dequeue하고 A/B 제품 결합은 Master가 수행 | inference_queue.py와 inspection_master |
| 촬영 재시도 | 같은 capture ID로 station 전체를 최대 2 attempts, 추가 대기 없음 | vision.capture.max_attempts |
| 추론 재시도 | 파일 read만 1회, timeout·model error는 재시도 없음 | inspection_vision/inference_queue.py |
| Error | timeout, file read, model error를 구분해 Log에 전달하며 모두 제품 NG 근거 | Vision ErrorCode와 Master |
| 이미지 형식 | 카메라 Mono8, 2448×2048 전체 ROI, canonical 1-channel PNG, RGB 변환 없음 | vision_capture.hardware.yaml |
| Trigger | Vision이 GigE Action1 즉시 command 발행, 외부 LED controller 상시점등 | vision_capture.hardware.yaml |
| Skew clock | PTP와 무관하게 host monotonic arrival max-min 사용 | vision.frame_arrival_skew_limit_us |
| 파일 삭제 | Vision은 완성 canonical 파일을 삭제하지 않으며 Log만 최근 10,000장 정책으로 삭제 | hardware.yaml의 Log 설정 |
| 종료 | waiting job 취소, active forward는 3초 soft timeout을 기록하되 완료까지 대기 | vision.shutdown.queue_drain_timeout_ms |
| 재시작 잔류 | Vision 종료 전에 PositionSettled·모터 정지가 이미 확인된 제품만 잔류 재촬영. 감속·이동 중 종료는 비잔류 실패 | Master·Control 통합 정책 |
| Replay | 같은 Master session의 durable Vision terminal만 replay | Vision spool과 ReplayStationResults |
| 보고서 | 정상 종료 때 제품 CSV, forward·제품시간·GPU·VRAM·timeout 통계 생성. 비정상 종료 CSV 복구와 정답 label 지표는 생성하지 않음 | inspection_log |

## Camera와 MVS

### 장치와 토폴로지

| 항목 | 현재 값 | 상태 | 남은 확인 | 수정 위치 |
|---|---|---|---|---|
| 카메라 모델 | MV-CS050-10GC | 확정 | SDK 열거 결과가 정확히 일치해야 함 | vision.camera.expected_model |
| Station A 순서 | DA9880512, DA9880516, DA7552836 | 초기값 | 각 serial의 실제 촬영면 역할과 모델 tensor view 순서를 확정 | vision.camera_ids.station_a |
| Station B | DA7838410 | 확정 | 실제 촬영면 역할 이름 기록 | vision.camera_ids.station_b |
| 카메라 IP | A: .13, .11, .14 / B: .12, 192.168.10.0/24 | 확정 | 실제 MVS 설정과 중복 IP가 없는지 확인 | vision.camera_network_map_json |
| 스위치 | NX-POE604GS 4포트 PoE | 확정 | 네 링크의 1 Gbps, PoE 안정성, 포트 오류 counter 확인 | OS·스위치 설정, ROS 파라미터 없음 |
| NIC | Realtek USB GbE Family Controller, Cat6 | 확정 | Ubuntu driver, 1 Gbps full duplex, USB 절전 해제 확인 | Ubuntu NetworkManager 또는 netplan |
| Host NIC IPv4 | 미정 | 결정 필요 | 192.168.10.0/24 안의 카메라와 겹치지 않는 고정 IP 및 /24 mask 결정 | Ubuntu NetworkManager 또는 netplan |
| 카메라 물리 역할 | 미정 | 결정 필요 | A1/A2/A3/B1의 설치 위치·방향을 serial에 영구 매핑 | station A/B camera list와 배포 문서 |

Station A camera list 순서는 모델 입력 view 순서로 취급합니다. 설치 역할과 이 순서가 다르면 모델 정확도가 무너질 수 있으므로 단순 serial 목록만으로는 완성되지 않습니다.

### SDK, firmware, Action1

| 항목 | 현재 값 | 상태 | 남은 확인 | 수정 위치 |
|---|---|---|---|---|
| Ubuntu | 24.04 x86_64 | 확정 | 실제 설치 PC에서 실행 검증 | OS 설치 |
| MVS 배포판 | HIKROBOT MVS 5.0.2 x86_64 | 확정 | Ubuntu 24.04에서 vendor Python sample import와 runtime load 확인 | 패키지 설치, YAML 값 아님 |
| Python binding | /opt/MVS/Samples/64/Python/MvImport | 확정 | 설치판의 실제 경로 확인 | vision.mvs.python_import_dir |
| MVS runtime | /opt/MVS/lib | 확정 | libMvCameraControl.so load 확인 | vision.mvs.runtime_root |
| runtime SDK version | SDK API에서 자동 기록 | 자동 조회 | 승인된 Linux runtime version과 비교할 allowlist가 필요하면 추가 | 현재 Log metadata, 별도 allowlist 없음 |
| camera firmware | 빈 기대값, 네 대의 실제 버전이 서로 같아야 통과 | 자동 조회 | 첫 Ubuntu 실행에서 네 serial별 버전을 기록하고 승인 여부 결정 | vision.camera.expected_firmware_version |
| firmware update | Vision이 자동 update하지 않음 | 확정 | 필요 시 HIKROBOT 승인 dav를 MVS로 수동 적용 | YAML 값 아님 |
| Action1 지원 | TriggerSource=Action1 설정과 ACK 성공이 필수 | 실험 필요 | 네 대 모두 node write, command ACK, frame 수신 시험 | 자동 초기화·capture 시험 |
| DeviceKey | 1 | 확정 | 현장 중복 Action sender가 없는지 확인 | vision.gige_action.device_key |
| A GroupKey/Mask | 1 / 1 | 확정 | A 세 대만 응답하는지 확인 | station_a.group_key, station_a.group_mask |
| B GroupKey/Mask | 2 / 2 | 확정 | B 한 대만 응답하는지 확인 | station_b.group_key, station_b.group_mask |
| Scheduled Action | false | 확정 | 즉시 Action만 사용 | vision.gige_action.scheduled |
| Broadcast | 255.255.255.255 | 초기값 | Ubuntu NIC routing에서 ACK IP 집합이 정확한지 확인 | vision.gige_action.broadcast_address |
| ACK timeout | 100 ms | 초기값 | 정상·부하 시험의 ACK p99.9를 보고 조정 | vision.gige_action.ack_timeout_ms |

Firmware 기대값이 비어 있으면 버전 검증을 생략하는 것이 아닙니다. 네 카메라의 자동 조회 버전이 모두 동일해야 하며, 한 대라도 비어 있거나 다르면 초기화가 실패합니다.

### Network, packet, 복구

| 항목 | 현재 값 | 상태 | 남은 시험·정책 | 수정 위치 |
|---|---|---|---|---|
| NIC MTU | 1500 | 확정 | Ubuntu에서 실제 MTU 확인 | OS NIC 설정 |
| GevSCPSPacketSize | 1500 | 확정 | 변경하지 않음 | vision.gige.packet_size |
| packet delay | 5000 ticks | 초기값 | A/B 동시 부하의 loss·skew·throughput로 조정 | vision.gige.packet_delay_ticks |
| packet loss 허용 | frame별 0 | 확정 | 장시간 시험에서 0인지 검증 | capture contract, 별도 threshold 없음 |
| A/B 동시 촬영 | NIC 대역폭이 충분할 때만 생산 허용 | 확정 조건부 | loss 0과 skew 분포 시험 전에는 허용하지 않음 | Master scheduling·통합시험, 현재 Vision toggle 없음 |
| resend | enabled | 초기값 | loss 원인을 숨기지 않도록 resend count도 분석 | vision.gige.packet_resend.* |
| bandwidth reserve | 현재 별도 설정 없음 | 결정 필요 | switch/NIC 시험 후 GevSCBWR 같은 reserve를 명시할지 결정 | 필요하면 hikrobot_mvs.py와 capture YAML에 추가 |
| firewall | 전용 NIC에서 MVS 통신 허용 필요 | 결정 필요 | Ubuntu 방화벽 사용 여부와 전용 subnet rule 확정 | ufw 또는 nftables, ROS 파라미터 없음 |
| reconnect 간격 | 1000 ms | 초기값 | cable pull/reinsert 시험 | vision.camera.reconnect_interval_ms |
| reconnect 최대 | 5회 | 초기값 | 현장 복구 시간과 비교 | vision.camera.reconnect_max_attempts |
| 지속 단절 PAUSE | 5000 ms | 초기값 | 5초가 생산 흐름에 적합한지 장애 시험 | vision.camera.disconnect_pause_after_ms |
| station 재연결 성공 | 필수 camera 전부 identity 검증 후 Action capture 성공 | 확정 | packet loss 0, 규격 frame, skew limit을 포함한 시험촬영 검증 | 코드 정책, skew 값만 YAML |

Queue full은 카메라 packet 문제가 아닙니다. 저장된 batch를 ENQUEUE_BLOCKED로 유지하고 Master가 PAUSED 상태에서 공간 회복을 기다립니다.

### Pixel과 timing

| 항목 | 현재 값 | 상태 | 남은 결정·시험 | 수정 위치 |
|---|---|---|---|---|
| Sensor ROI | 2448×2048, offset 0,0 | 확정 | 네 대 모두 실제 frame 크기 확인 | vision.image.sensor_width, sensor_height; MVS adapter |
| Camera pixel format | Mono8 | 확정 | MVS에서 Mono8 node 설정 성공 확인 | hikrobot_mvs.py, 현재 별도 YAML key 없음 |
| Canonical file | MONO8_PNG | 확정 | OpenCV/Pillow 재열기와 digest 검증 | vision.image.canonical_pixel_format |
| Bayer, white balance, demosaic | 사용하지 않음 | 해당 없음 | RGB 모델로 정책 변경할 때만 재검토 | 현재 파라미터 없음 |
| Exposure | 미정 | 실험 필요 | motion blur, saturation, 불량 분리 성능으로 결정 | 아직 key 없음; 결정 후 capture YAML과 hikrobot_mvs.py에 추가 |
| Gain | 미정 | 실험 필요 | SNR과 조명 편차로 결정 | 아직 key 없음; 결정 후 capture YAML과 hikrobot_mvs.py에 추가 |
| PNG compression | 3 | 초기값 | 저장시간·CPU·용량 시험 | vision.image.png_compression_level |
| Acquisition timeout | 0, hardware 시작 차단 | 실험 필요 | 촬영 시간 분포와 저장·검증 시간을 기준으로 생산값 결정 | vision.capture.acquisition_timeout_ms |
| Host arrival 기록 | MV_CC_GetImageBuffer 성공 직후 monotonic_ns | 확정 | 변경하지 않음 | hikrobot_mvs.py |
| Camera raw timestamp | raw ticks와 DEVICE_TICKS_UNSYNCED domain만 진단 저장 | 확정 | unit·wrap을 생산 skew에 사용하지 않음 | 코드 metadata |
| PTP | false, 생산 판정에 미사용 | 확정 | 지원 시험은 진단·향후 개선용이며 완성 차단값 아님 | vision.ptp.enabled |
| Arrival skew limit | 0, hardware 시작 차단 | 실험 필요 | A와 B 분포를 수집하고 production upper limit 승인 | vision.frame_arrival_skew_limit_us |
| Late frame | metadata만 보존, 공정 판정에 사용하지 않음 | 확정 | 별도 late image 경로 없음 | LogEvent 정책 |

Acquisition timeout 후보 통계는 평균 acquisition×2 + save overhead + validation으로 기록하지만, 평균값만으로 생산 timeout을 확정하지 말고 tail 분포와 장애 복구 시간을 함께 확인합니다.

## Queue와 Worker

| 항목 | 현재 값 | 상태 | 남은 결정·시험 | 수정 위치 |
|---|---|---|---|---|
| Queue capacity | 16 jobs | 초기값 | 최대 제품 유입속도와 최악 latency로 검증 | vision_runtime.hardware.yaml의 vision.queue.capacity |
| A total timeout | 0, hardware 시작 차단 | 실험 필요 | A enqueue→결과 p99.9×1.2, 최소 정상 표본 10,000 | vision.inference.station_a.total_timeout_ms |
| B total timeout | 0, hardware 시작 차단 | 실험 필요 | B enqueue→결과 p99.9×1.2, 최소 정상 표본 10,000 | vision.inference.station_b.total_timeout_ms |
| 공통 timeout fallback | 0 | 확정 | station별 값 사용 후에도 0 유지 권장 | vision.inference_queue_total_timeout_ms |
| Worker 수 | 1 | 초기값 | latency, GPU utilization, VRAM 시험 후 조정 | vision_model.hardware.yaml의 vision.worker_count |
| Model lock | true | 초기값 | 현재 worker 1에서는 실질 경쟁 없음. worker 증가 시 thread safety 검증 | vision.model.serialize_access |
| Queue full | ENQUEUE_BLOCKED 후 Master PAUSE | 확정 | 최대 정지시간과 operator recovery 시험 | 코드 정책 |
| RAM saturation | path-only queue라 별도 Vision RAM limit 없음 | 결정 필요 | 장시간 시험 후 system RAM alarm이 필요하면 공통 health 정책 추가 | 현재 key 없음 |
| GPU device | 단일 GPU index 0 사용 전제 | 결정 필요 | 실제 backend의 명시값 cuda:0 승인 | backend 구현 때 vision.model.device key 추가 권장 |
| CPU/GPU affinity | 미설정 | 결정 필요 아님 | 성능 jitter가 확인될 때만 systemd/launch에서 지정 | launch 또는 systemd |

Capacity 검증 시작식은 다음과 같습니다.

    capacity >= ceil(lambda_job × T_worst_seconds × safety_factor)

lambda_job은 최대 제품 유입률에 제품당 실제 station job 수를 곱한 값입니다. A job은 항상 1개이고 B job은 A가 terminal NG가 되기 전에 B 촬영까지 진행된 비율만큼 발생합니다. model lock이 활성화되면 worker 수를 늘려도 forward 구간은 직렬이므로 처리율 증가로 계산하면 안 됩니다.

## Model, 전처리, 판정

### 현재 고정·초기 설정

| 항목 | 현재 값 | 상태 | 수정 위치 |
|---|---|---|---|
| Runtime | PYTORCH_TORCHSCRIPT | 확정 | vision.model.runtime |
| Artifact path | /opt/inspection/models/Model_v_1.pt | 초기값 | vision.model.path |
| Version | Model_v_1 | 확정 형식 | vision.model.version |
| A/B artifact | 두 station이 같은 pt와 SHA-256 사용 | 확정 | 단일 model backend |
| SHA-256 | 빈 값, hardware 시작 차단 | 모델 주입 필요 | vision.model.sha256 |
| CUDA required | true, CPU fallback 금지 | 확정 | vision.model.cuda_required |
| Warmup | 10회 | 초기값 | vision.model.warmup_runs |
| Station A batch | 3 images | 확정 | 모델 backend |
| Station B batch | 1 image | 확정 | 모델 backend |

SHA-256은 모델 파일 내용의 지문입니다. Ubuntu에서 다음 명령으로 계산합니다.

    sha256sum /opt/inspection/models/Model_v_1.pt

한 session 동안 path, version, SHA-256과 config fingerprint가 고정됩니다. PAUSED 또는 INITIALIZING 상태에서만 다음 실행용 모델을 교체합니다.

### 실제 모델 담당자가 반드시 제공할 계약

| 항목 | 정확히 필요한 내용 | 주입 위치 |
|---|---|---|
| 파일 형식 | 실제 pt가 torch.jit.load 가능한 TorchScript인지 확인 | model_backend.py |
| 입력 dtype | 예: float32 | model_backend.py와 model YAML 신규 key |
| 입력 shape | A=[3,1,H,W], B=[1,1,H,W]의 H,W | model_backend.py와 model YAML 신규 key |
| Camera order | A batch index 0,1,2와 실제 serial·촬영면 대응 | capture camera list와 preprocessor |
| Load | Mono8 PNG를 1-channel로 읽는 정확한 API와 range | model_backend.py |
| Crop | 좌·상·우·하 좌표 또는 ROI 계산식 | model YAML 신규 key |
| Resize | 종횡비 유지 방식, 목표 H,W, letterbox 또는 crop, interpolation | model YAML 신규 key |
| Padding | pad 위치와 값 | model YAML 신규 key |
| Scale | uint8 0..255를 어떤 범위로 바꾸는지 | model YAML 신규 key |
| Normalize | channel mean/std 또는 다른 수식 | model YAML 신규 key |
| Output schema | tensor·tuple·dict 중 무엇이며 shape와 각 index 의미 | model_backend.py |
| Score | score 범위, 값이 클수록 NG인지 PASS인지 | model_backend.py와 interfaces 문서 |
| Threshold | view별 또는 station별 임계값과 경계값 포함 규칙 | model YAML 신규 key |
| 결합식 | view NG 하나면 station NG는 확정. station score 대표값 계산식은 별도 확정 | model_backend.py |
| 불확실 | threshold 인접 구간이나 confidence 부족을 NG, 오류, 재촬영 중 무엇으로 처리할지 | 정책 확정 후 model_backend.py |
| 비유한 출력 | NaN·Inf는 inference failure와 제품 NG 근거 | 확정, vision_node.py 검증 |
| OOM | 현재 제품 inference failure, CPU fallback 없이 PAUSE·재초기화 | 확정, worker와 Master |

현재 hardware backend는 위 계약이 없기 때문에 의도적으로 READY가 되지 않습니다. 이 표의 내용을 받으면 UnconfiguredTorchScriptModel을 실제 backend로 교체할 수 있습니다.

## 파일, 저장장치, 보존

| 항목 | 현재 값·정책 | 상태 | 수정 위치 |
|---|---|---|---|
| Image root | /var/lib/inspection/images | 확정 | vision.data_root |
| Canonical naming | raw/station_ID/frame_batch_uuid/serial.png | 확정 | 변경 시 hikrobot_mvs.py |
| Temporary file | 숨김 part 파일에 쓰고 fsync 후 os.replace | 확정 | hikrobot_mvs.py |
| Digest | 완성 PNG SHA-256 | 확정 | capture contract |
| Vision spool | /var/lib/inspection/spool/vision.sqlite3 | 확정 | vision.result_spool_path |
| Log DB | /var/lib/inspection/log/inspection.sqlite3 | 확정 | hardware.yaml의 log.database_path |
| Report root | /var/lib/inspection/reports | 확정 | hardware.yaml의 log.report_root |
| Disk warning | 사용률 90% | 확정 | vision.disk.warning_ratio |
| Disk stop | 사용률 95%, 신규 촬영 중지·PAUSE | 확정 | vision.disk.stop_ratio |
| Canonical retention | 최근 10,000장 | 확정 | log.image_retention.max_completed_images |
| 삭제 소유권 | Log만 data_root 내부 경로를 삭제 | 확정 | inspection_log |
| 전처리 이미지 | 저장하지 않음 | 확정 | model backend |
| Memory cache | terminal을 durable spool에 기록한 뒤 완료 job을 해제하고, spool record는 Log ACK까지 유지 | 확정 | worker, DurableLogSpool |
| Late frame | metadata만 기록 | 확정 | LogEvent |
| Service account | 미정 | 결정 필요 | systemd 사용자와 디렉터리 chown |
| 파일 권한 mode | OS 기본 umask 의존 | 결정 필요 | 배포 systemd의 User, Group, UMask |
| 삭제 handshake | Vision 요청 없이 Log가 durable metadata 기준으로 직접 retention 실행 | 현재 정책 | 별도 삭제 ACK가 필요하면 interface 변경 검토 |

완성 canonical 파일은 Vision이 임의 삭제하지 않습니다. 다만 atomic rename 전 실패한 part 파일과 capture attempt 내부의 불완전 artifact 정리는 Vision이 수행할 수 있습니다.

## 자동 timeout 적용

| 항목 | 현재 값 | 수정 위치 |
|---|---|---|
| Log 후보 계산 | station별 p99.9×1.2 | hardware.yaml의 log.timeout_tuning.* |
| 최소 표본 | A/B 각각 10,000 | log.timeout_tuning.minimum_samples |
| Log 자동 생성 | false | log.timeout_tuning.auto_apply |
| 후보 파일 | /var/lib/inspection/config/vision_timeout_tuning.json | log.timeout_tuning.generated_path |
| Vision 자동 읽기 | false | vision_runtime.hardware.yaml의 vision.timeout_tuning.auto_apply |
| 동일성 guard | model version, SHA-256, camera·worker·lock config fingerprint 일치 | 코드 정책 |

자동 적용을 사용하려면 Log와 Vision의 auto_apply를 모두 true로 해야 합니다. 기본값 false에서는 통계만 기록하고 다음 실행 YAML을 바꾸지 않습니다. 자동값은 정상 종료 session만 사용합니다.

## 지금 반드시 제공하거나 시험해야 할 최소 묶음

### 모델 담당자 제공

- 실제 TorchScript pt와 SHA-256
- A camera 물리 view 순서
- 입력 H,W·dtype·crop·resize·padding·scale·normalization
- 출력 구조·score 의미·threshold·station score 계산·불확실 처리
- cuda:0 사용 승인과 batch 3 VRAM·latency 시험 결과

### 카메라 장비 시험

- Ubuntu MVS에서 자동 조회된 네 firmware와 runtime SDK version
- Host NIC 고정 IPv4, /24 mask, route와 firewall 정책
- 네 카메라 Action1 지원·ACK·2448×2048 Mono8 저장 성공
- exposure, gain, acquisition timeout 생산값
- packet delay 5000에서 단독·A/B 동시 packet loss 0과 skew 분포
- frame_arrival_skew_limit_us 생산값
- cable disconnect/reconnect와 5초 PAUSE 정책 인수 결과

### 공정·성능 시험

- 최대 제품 유입 속도
- A/B enqueue→결과 최소 10,000 표본과 p99.9
- queue 16, worker 1, lock true의 backlog·GPU·VRAM·온도
- 저장속도, 90/95% disk 동작, Log의 10,000장 삭제
- 정상 종료·비정상 종료·Vision/Log 재시작·Sensor3 late-result 시나리오

## 파라미터를 수정하는 방법

소스 저장소의 YAML을 수정합니다. colcon build 후 install 아래에 복사된 파일을 직접 수정하면 다음 빌드에서 사라질 수 있으므로 사용하지 않습니다.

| 종류 | 수정할 원본 파일 |
|---|---|
| Camera, Action, network, firmware, pixel, capture | ros2_ws/src/basic_packages/inspection_bringup/config/vision_capture.hardware.yaml |
| Model identity, CUDA, warmup, worker, lock | ros2_ws/src/basic_packages/inspection_bringup/config/vision_model.hardware.yaml |
| Queue, station timeout, disk, spool, shutdown, timeout apply | ros2_ws/src/basic_packages/inspection_bringup/config/vision_runtime.hardware.yaml |
| Log DB, report, retention, timeout 후보 생성 | ros2_ws/src/basic_packages/inspection_bringup/config/hardware.yaml |
| 아직 key가 없는 전처리·threshold·exposure·gain | 먼저 vision_node.py에 declare/validation을 추가하고 해당 hardware YAML에 입력 |
| OS NIC, firewall, service account, file permission | Ubuntu NetworkManager/netplan, ufw/nftables, systemd unit |

Launch 적용 순서는 hardware.yaml 다음 capture, model, runtime입니다. Vision 파라미터는 실행 중 동적 변경을 전제로 구현되지 않았으므로 값을 바꾼 뒤 프로그램을 정상 종료하고 다시 시작합니다.

설정 변경 후 최소 검증 순서는 다음과 같습니다.

    cd ros2_ws
    source /opt/ros/jazzy/setup.bash
    python3 tools/verify_skeleton.py
    python3 tools/test_domain_contracts.py
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch inspection_bringup inspection_system.launch.py profile:=hardware

0 또는 빈 값은 실수로 남은 기본값이 아니라 생산 운전을 막는 fail-closed 표시입니다. 시험 없이 임의 값으로 채워 READY를 우회하지 않습니다.

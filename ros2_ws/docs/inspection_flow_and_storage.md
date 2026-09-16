# 검사 흐름 및 결과 저장 위치

이 문서는 hardware profile과 배포 모델 `MB_v3_robust`를 기준으로 합니다.

## 전체 흐름

```mermaid
flowchart LR
    S1[Sensor 1 감지] --> F[Master 제품 ID·FIFO 등록]
    F --> PA[Mega 자율 이송·Station A 정지]
    PA --> PSA[PositionSettled A]
    PSA --> CA[CaptureProduct A]
    CA --> RA[CAM_A_1/2/3 Mono8 원본 저장]

    S2[Sensor 2 감지] --> PB[Mega 자율 이송·Station B 정지]
    PB --> PSB[PositionSettled B]
    PSB --> CB[CaptureProduct B]
    CB --> RB[CAM_B_1 Mono8 원본 저장]

    RA --> Q[path-only inference queue]
    RB --> Q
    Q --> P[artifact preprocessing_by_view 로드<br/>V threshold → 8-connected 최대 component → crop_2]
    P --> M[black padding·RGB 복제·ImageNet normalize]
    M --> I[view별 PatchCore V3 추론]
    I --> V[artifact threshold·margin으로 view PASS/NG]
    V --> ST[view 하나라도 NG이면 station NG]
    ST --> DS[Vision SQLite spool에 durable terminal 기록]
    DS --> PUB[StationResult 발행·Log DB 반영]
    ST -->|NG view| HM[crop_1·crop_2·heatmap·overlay·위치 JSON 저장]

    PUB --> LK[Sensor 3 시점 Master 최종 판정 잠금]
    S3[Sensor 3 감지] --> LK
    LK --> OUT[/inspection/master/product_result_locked]
    LK --> ACT[PASS_THROUGH 또는 DIVERT_NG]
    LK --> REP[최종 결과 DB·세션 보고서]
```

Station A 추론은 이동과 비동기로 진행됩니다. Sensor 3에서 A/B 결과가 아직 없거나
촬영·추론이 최종 실패한 경우 최종 판정은 `FORCED_NG`입니다.

## Hardware 저장 위치

| 내용 | 위치 | 핵심 필드/파일 |
|---|---|---|
| 촬영 원본 | `/var/lib/inspection/images/raw/station_<1|2>/product_<fifo 6자리>_<frame_batch_id>/` | 카메라 serial 이름의 Mono8 PNG |
| NG view 진단 | `/var/lib/inspection/images/diagnostics/ng/<frame_batch_id>/<CAM_VIEW>/` | `crop_1.png`, `crop_2.png`, `anomaly_heatmap.png`, `anomaly_overlay.png`, `anomaly_metadata.json` |
| 전처리 실패 진단 | `/var/lib/inspection/images/diagnostics/preprocessing_failure/<frame_batch_id>/<CAM_VIEW>/` | `crop_1.png`, `crop_2.png` |
| 전체 영속 로그/결과 | `/var/lib/inspection/log/inspection.sqlite3` | `product_terminal_results`, `vision_station_terminals`, `log_events` |
| Vision 미전송 결과 spool | `/var/lib/inspection/spool/vision.sqlite3` | Log ACK 전 durable station terminal |
| Master 미전송 로그 spool | `/var/lib/inspection/spool/master.sqlite3` | Log ACK 전 Master event |
| 세션 결과 CSV | `/var/lib/inspection/reports/<session_id>/product_inference_results.csv` | 최종 PASS/NG/FORCED_NG, A/B 판정·오류·시간 |
| 세션 요약 | `/var/lib/inspection/reports/<session_id>/session_summary.json` | 처리량·시간·GPU/VRAM 요약 |

`anomaly_metadata.json`의 좌표계는 해당 view의 `crop_2` 픽셀입니다.
`anomaly_location.peak_xy`는 최대 anomaly 지점, `threshold_exceeding_bbox_xywh`는
threshold 이상 영역의 bounding box입니다. 파일에는 `view_score_raw`,
`view_threshold`, `view_score_normalized`도 같이 들어갑니다.

Station 단위 점수는 SQLite `vision_station_terminals.score`에 있고, view별 점수는
같은 row의 JSON 경로 `$.payload.view_raw_scores`와 `$.payload.view_scores`에
있습니다. 후자는 각각 raw score와 artifact threshold로 나눈 normalized
score입니다.

정상 제품은 별도 OK 이미지 복사본을 만들지 않습니다. 원본 PNG와 DB/CSV 결과가
정상 기록입니다. NG일 때만 진단 이미지가 추가됩니다. `vision_station_terminals`의
`payload_json`에는 view별 이름·판정·raw/normalized score, 원본 이미지 경로,
진단 파일 경로와 실제 model identity가 함께 보존됩니다.

## 전처리 기준의 단일 출처

운영 전처리 기준은 `/opt/inspection/models/MB_v3_robust/manifest.json`의
`preprocessing_by_view`입니다. 현재 A 3개 view의 `v_threshold`는 38, B view는
23이며, connectivity 8, 최대 component만 유지, 연결성 강제 검사는 비활성입니다.
코드나 ROS YAML에 별도 threshold fallback을 두지 않습니다.

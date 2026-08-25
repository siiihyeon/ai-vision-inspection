#!/usr/bin/env bash
# Master의 /inspection/master/operator_command Service를 감싸는 터미널 wrapper.
# HMI 없이 터미널만으로 운전하기로 한 프로젝트 방향에 맞춘 개발/운전용 스크립트입니다.
# request_id(uuid)를 매번 손으로 안 만들어도 되게 하는 것 외에는 아무 로직도 없습니다 -
# 실제 명령 처리는 전부 Master의 OperatorCommand 핸들러가 합니다.
set -euo pipefail

usage() {
  echo "Usage: $0 {init|start|pause|resume|reset|line-clear} [reason]" >&2
  echo "  OPERATOR_ID 환경변수로 operator_id를 지정할 수 있습니다 (기본값: \$USER)." >&2
  exit 1
}

[ "$#" -ge 1 ] || usage

case "$1" in
  init)       TYPE=1 ;;
  start)      TYPE=2 ;;
  pause)      TYPE=3 ;;
  resume)     TYPE=4 ;;
  reset)      TYPE=5 ;;
  line-clear) TYPE=6 ;;
  *) usage ;;
esac

REASON="${2:-operator $1 via op}"
OPERATOR_ID="${OPERATOR_ID:-${USER:-operator}}"

ros2 service call /inspection/master/operator_command \
  inspection_interfaces/srv/OperatorCommand \
  "{request_id: '$(uuidgen)', command_type: ${TYPE}, reason: '${REASON}', operator_id: '${OPERATOR_ID}'}"

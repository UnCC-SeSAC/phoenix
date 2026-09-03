#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
test_root="$(mktemp -d)"
owned_pid=""
unrelated_pid=""
cleanup() {
    [[ -z "$owned_pid" ]] || kill -KILL -- "-$owned_pid" 2>/dev/null || true
    [[ -z "$unrelated_pid" ]] || kill "$unrelated_pid" 2>/dev/null || true
    rm -rf "$test_root"
}
trap cleanup EXIT

mkdir -p "$test_root/bin" "$test_root/log/run"
printf '%s\n' "$test_root/log/run" >"$test_root/log/current_run"

cat >"$test_root/bin/docker" <<'FAKE_DOCKER'
#!/usr/bin/env bash
set -euo pipefail
run_dir=""
for argument in "$@"; do
    case "$argument" in
        RUN_LOG_DIR=*) run_dir="${argument#RUN_LOG_DIR=}" ;;
    esac
done
command="${!#}"
if [[ "$command" == */current_run ]]; then
    exec cat "$command"
fi
if [[ "$command" == *'for file in "$RUN_LOG_DIR"/*.pgid'* ]]; then
    RUN_LOG_DIR="$run_dir" exec bash -c "$command"
fi
if [[ "$command" == *"ros2 service call"* || "$command" == *"ros2 topic pub"* ]]; then
    [[ "$command" == *"timeout -k 1s 3s"* ]]
    timeout 0.1s sleep 5 || true
fi
exit 0
FAKE_DOCKER
chmod +x "$test_root/bin/docker"

setsid bash -c 'sleep 60 & echo $! >"$1"; wait' bash "$test_root/child.pid" &
owned_pid=$!
sleep 60 &
unrelated_pid=$!
sleep 0.2
printf '%s\n' "$owned_pid" >"$test_root/log/run/base.pid"
printf '%s\n' "$owned_pid" >"$test_root/log/run/base.pgid"

started_at="$(date +%s)"
PATH="$test_root/bin:$PATH" VLA_E2E_LOG_ROOT="$test_root/log" \
    bash "$repo_root/scripts/vla_hardware_e2e.sh" stop >/dev/null
elapsed=$(( $(date +%s) - started_at ))

[[ "$elapsed" -lt 10 ]]
owned_state="$(ps -o stat= -p "$owned_pid" 2>/dev/null || true)"
child_pid="$(cat "$test_root/child.pid")"
child_state="$(ps -o stat= -p "$child_pid" 2>/dev/null || true)"
[[ -z "$owned_state" || "$owned_state" == Z* ]]
[[ -z "$child_state" || "$child_state" == Z* ]]
kill -0 "$unrelated_pid"

echo "wrapper owned-process stop test: PASS"

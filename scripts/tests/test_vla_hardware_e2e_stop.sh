#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
test_root="$(mktemp -d)"
owned_pid=""
stubborn_pid=""
zombie_owner_pid=""
unrelated_pid=""
cleanup() {
    [[ -z "$owned_pid" ]] || kill -KILL -- "-$owned_pid" 2>/dev/null || true
    [[ -z "$stubborn_pid" ]] || kill -KILL "$stubborn_pid" 2>/dev/null || true
    [[ -z "$zombie_owner_pid" ]] || kill -KILL -- "-$zombie_owner_pid" 2>/dev/null || true
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
if [[ -n "$run_dir" && "$command" == *"owned_processes.snapshot"* ]]; then
    RUN_LOG_DIR="$run_dir" exec bash -c "$command"
fi
if [[ "$command" == *"ros2 service call"* || "$command" == *"ros2 topic pub"* ]]; then
    [[ "$command" == *"timeout -k 1s 3s"* ]]
    timeout 0.1s sleep 5 || true
fi
exit 0
FAKE_DOCKER
chmod +x "$test_root/bin/docker"

setsid python3 -c 'import os,signal,sys,time; child=os.fork(); open(sys.argv[1], "w").write(str(child)); signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); time.sleep(60)' "$test_root/child.pid" &
owned_pid=$!
setsid python3 -c 'import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)' &
stubborn_pid=$!
setsid python3 -c 'import os,sys,time; child=os.fork(); open(sys.argv[1], "w").write(str(child)); os._exit(0) if child == 0 else time.sleep(60)' "$test_root/zombie.pid" &
zombie_owner_pid=$!
sleep 60 &
unrelated_pid=$!
sleep 0.2
printf '%s\n' "$owned_pid" >"$test_root/log/run/base.pid"
printf '%s\n' "$owned_pid" >"$test_root/log/run/base.pgid"
printf '%s\n' "$stubborn_pid" >"$test_root/log/run/stubborn.pid"
printf '%s\n' "$stubborn_pid" >"$test_root/log/run/stubborn.pgid"
printf '%s\n' "$zombie_owner_pid" >"$test_root/log/run/zombie_owner.pid"
printf '%s\n' "$zombie_owner_pid" >"$test_root/log/run/zombie_owner.pgid"

started_at="$(date +%s)"
PATH="$test_root/bin:$PATH" VLA_E2E_LOG_ROOT="$test_root/log" \
    bash "$repo_root/scripts/vla_hardware_e2e.sh" stop >/dev/null
elapsed=$(( $(date +%s) - started_at ))

[[ "$elapsed" -lt 12 ]]
owned_state="$(ps -o stat= -p "$owned_pid" 2>/dev/null || true)"
child_pid="$(cat "$test_root/child.pid")"
child_state="$(ps -o stat= -p "$child_pid" 2>/dev/null || true)"
zombie_pid="$(cat "$test_root/zombie.pid")"
[[ -z "$owned_state" || "$owned_state" == Z* ]]
[[ -z "$child_state" || "$child_state" == Z* ]]
! kill -0 "$stubborn_pid" 2>/dev/null
! grep -qx "$owned_pid" "$test_root/log/run/final_kill.pids"
grep -qx "$stubborn_pid" "$test_root/log/run/final_kill.pids"
! grep -qx "$zombie_pid" "$test_root/log/run/final_kill.pids"
kill -0 "$unrelated_pid"

echo "wrapper owned-process stop test: PASS"

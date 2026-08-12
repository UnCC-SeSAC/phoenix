# Project Working Rules

## Project Goal

This repository builds a ROS 2 fire-exploration robot. The VLA work integrates a
Semantic WorldModel and decision layer with Perception, SLAM, Nav2, and robot
hardware. The authoritative navigation and semantic space is a 2D `map` frame.

## Branch Ownership

- `feature/vla-brain`: latest standalone VLA module release/verification branch.
- `integration/vla-robot-e2e`: latest VLA plus other team modules and robot E2E
  integration.
- Classify every change before editing. Ask: "Is this needed when VLA is used by
  itself?"
- If yes, it is VLA-owned and both branches must retain the same current VLA
  semantics. Examples: `fire_vla_core`, Domain/Core, WorldModel, Resolver,
  Validator, Dispatcher, Qwen adapter, VLA ROS/perception adapters, VLA UI,
  standalone VLA launch/config, and VLA tests.
- If no, it is integration-only. Examples: `image_pipeline` implementation,
  SLAM/Nav2 source, robot hardware, Pi/Docker runtime, DDS integration,
  team-wide composition, and hardware-specific configuration. Keep these only
  on `integration/vla-robot-e2e`.
- Do not backport integration-only changes to `feature/vla-brain`.

## Upstream Ownership

- Treat team-owned branches/packages such as `albitro/image_pipeline`, SLAM/Nav2
  branches, and hardware code as their owners' source of truth.
- Inspect the latest source and contract before adapting VLA.
- Prefer absorbing contract differences at the Adapter boundary; do not modify
  upstream for VLA convenience.
- Do not blindly merge an entire team branch. Review its diff and import only
  the required, semantically compatible range.

## Architecture

- Preserve Clean Architecture and Port–Adapter boundaries.
- Do not add ROS dependencies to Domain/Core.
- Handle ROS messages, topics, CameraInfo, and TF in Adapter layers.
- Keep WorldModel semantic state in 2D `map` coordinates.
- General obstacle avoidance belongs to Nav2.
- LLM output is not authoritative physical state. ROS/WorldModel owns position,
  confidence, observations, and execution results.
- Prefer minimal changes and reuse verified components. Do not add a Manager,
  Event Bus, or abstraction without a demonstrated need.

## Navigation Ownership

- DETERMINISTIC mode uses Frontier/StateManager/MissionExecutor-family goal
  ownership.
- VLA mode uses VLA Brain and VLA Navigation Bridge ownership.
- Never configure multiple independent owners to send goals concurrently to
  `/navigate_to_pose`.

## Safety

- Before authorized motion, verify runtime state and preflight conditions.
- Reject navigation on stale robot pose, invalid TF/depth, or invalid map pose.
- Keep autonomous goal senders mutually exclusive.
- Generate real motion only when the task explicitly authorizes it.
- On unexpected motion: cancel the active goal, confirm velocity stop, then send
  explicit motor zero through the verified halt path.
- Do not send a real Pump command unless the hardware boundary is connected and
  the task explicitly authorizes it.
- In hardware-free work, never issue Robot, Nav2 motion, motor, or Pump commands.

## Perception Boundary

- `albitro/image_pipeline` is the current upstream perception source of truth.
- Do not duplicate its full schema here; inspect upstream source and official
  architecture documentation.
- Responsibility boundary: image_pipeline provides pixel, depth, and source
  timestamp; the VLA ROS Adapter performs CameraInfo projection and source-time
  TF conversion to canonical 2D `map (x,y)` SemanticObservation.

## Documentation

- Keep official architecture, contracts, and reports in `docs/`.
- Keep one-off Codex tasks in `docs/private/codex_tasks/` and session handoffs in
  `docs/private/handoffs/`. Never track `docs/private/`.
- When production architecture or a contract changes, update the relevant
  official documentation in the same task and preferably the same push.
- Do not put session logs, credentials, transient SHAs/Issues, full payloads, or
  long hardware procedures in this file.

## Git and GitHub

- Start with `git status`, `git fetch origin --prune`, and current remote SHAs.
- Never push directly to `main`, force-push, or push to another team's branch.
- `feature/vla-brain` may receive verified standalone VLA commits and pushes;
  open a PR only when the team explicitly requests an actual merge.
- `integration/vla-robot-e2e` may receive verified integration/E2E commits and
  pushes. Do not automatically request review or open a PR to `main`.
- Merging to `main` always requires separate user approval.
- Preserve user files and untracked artifacts; never delete them casually.
- Do not resolve conflicts by blindly choosing `ours` or `theirs`; resolve the
  semantic contract.
- Commit and push only after proportionate verification passes. Keep related
  code and documentation in the same task.
- Default work tracking is: Issue → implementation → verification → commit and
  push → result comment → close only when Acceptance Criteria are met. A PR is
  not a required step.
- Write Issue titles, bodies, and result comments in concise, professional
  Korean. Keep only necessary technical names, topics, classes, and commands in
  English; do not paste long execution logs.
- PR-related write actions require an explicit user request. Without it, do not
  create a PR or Draft PR, comment on an existing PR, mark it ready for review,
  or merge it. A completed commit, Issue, or integration verification does not
  imply PR authorization.
- Create a PR only when the user explicitly asks to merge a specific branch into
  another shared branch or `main`. Use a concise professional Korean title and
  body covering purpose, key changes, verification, and remaining work.

## Verification

- Run focused tests, relevant full pytest, relevant colcon builds, launch/module
  syntax checks, and `git diff --check` in proportion to the change.
- Preserve existing regressions. Never claim hardware PASS from a software-only
  test; label untested physical work `HARDWARE_PENDING`.

## Stop Conditions

Stop and report instead of guessing when the upstream contract is unclear,
tracked modifications are unexpected, hardware identity is uncertain, motion
may be unsafe, a semantic merge conflict remains, the live interface differs
from documentation, or the required architecture exceeds user authorization.

## Detailed References

- `docs/VLA_SYSTEM_ARCHITECTURE_GUIDE.md`
- `docs/CURRENT_VLA_DATA_ARCHITECTURE.md`
- `docs/VLA_ROBOT_RUNTIME_HANDSON.md`
- `docs/INTEGRATION_REPORT.md`
- `docs/VERIFICATION_REPORT.md`

Read private task/handoff documents only when the current work requires them.

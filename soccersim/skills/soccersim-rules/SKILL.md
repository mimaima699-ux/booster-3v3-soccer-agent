---
name: soccersim-rules
description: Use when editing or reviewing SoccerSim 3v3 strategy, restart behavior, legal positioning, GameController parsing, referee penalty handling, or tests that depend on match rules in this repository.
---

# SoccerSim Rules

Use this skill before changing code that can affect rule compliance, especially strategy targets, Ready/Set positioning, restart behavior, shooting decisions, penalty handling, or GameController state parsing.

## Authority

- Repository root: `/home/ylb/BoosterStudioProjects/SoccerSim`
- Authoritative local summary: `note/3v3_rule.md`
- Upstream referee implementation reference:
  `/home/ylb/booster_robotics/booster_studio/booster_studio_client/booster_robocup_sim/docs/automatic_game_control.md`

When rule documents disagree, follow `note/3v3_rule.md`; if exact referee behavior matters, compare against the upstream referee document or code.

## Rule-Aware Coding Workflow

1. Read `note/3v3_rule.md` before editing strategy or referee-facing code.
2. Identify the active phase: `READY`, `SET`, `PLAYING`, `stopped=true`, kick-off, or another set play.
3. Preserve Pylance/pyright-friendly types. Narrow external GameController/ROS/JSON values before passing them to typed APIs.
4. Keep targets legal by construction rather than relying on the referee to punish illegal positions.
5. Add or update focused tests when behavior changes. Common files:
   - `tests/test_soccer_framework_core.py`
   - `tests/test_soccer_framework_runtime.py`
   - `tests/test_soccer_framework_telemetry.py`
   - `tests/test_auto_debug_match.py`
6. If validating in simulation or analyzing `artifacts/auto_debug`, also use `skills/soccersim-auto-debug/SKILL.md`.

## Strategy Guardrails

- Ready/Set: keep all robots inside the field and in their own half unless a kick-off main-kicker exception is intentionally used.
- Set: do not command movement after Set is active; motion over `0.08 m` can trigger `motionInSet`.
- Kick-off: do not rely on the obsolete "different teammate touches" rule. Current legality is 2 stable touches total or set play expiry.
- Opponent restarts: keep non-kicking robots at least `1.45 m` from the ball; goal-kick also excludes the kicking side's defensive penalty area.
- Own restarts: avoid immediate shots from `throwIn` or `indirectFreeKick` unless the indirect requirement is already satisfied.
- `stopped=true`: treat as a no-motion period in `PLAYING`; visible movement over `0.3 m` can map to `motionInSet`.
- Stuck ball: current referee clears the ball after about `700 ms`; do not model it as a penalty on the nearest robot.
- Leaving field and excess goal-area defenders are currently default-disabled, but strategy should still prefer legal, robust field positions.

## Files To Inspect

- `src/soccer_framework/types.py`: typed soccer data (`PlayContext`, `RobotCommand`, `GameControlState`), `PlayContextProvider` abstract, protocol constants.
- `src/soccer_framework/config.py`: `SoccerConfig` and `SoccerStrategyTuning`, including env-var loading.
- `src/soccer_framework/game_state.py`: GameController JSON encode/decode.
- `src/soccer_framework/ros_truth.py` / `robot.py` / `game_controller.py`: ROS truth provider, robot/kick adapters, GC topic bridge.
- `src/runtime.py`: `SoccerTeamRuntime` assembly (framework adapters + strategy + behavior tree control loop).
- `src/soccer_framework/__init__.py`: public data and command API for strategy developers.
- `src/tactics/geometry.py`: field-frame helpers, coordinate transforms, and legal target clamping.
- `src/tactics/targeting/restart.py`: restart avoidance targets.
- `src/strategy.py`: strategy behavior entry point.
- `src/trees/`: behavior tree data, condition, and action nodes.

## Verification

For documentation-only edits, run:

```bash
git diff --check -- note/3v3_rule.md skills/soccersim-rules/SKILL.md
```

For strategy/rules code edits, prefer:

```bash
python3 -m py_compile src/__init__.py src/main.py src/runtime.py src/soccer_framework/__init__.py src/soccer_framework/types.py src/soccer_framework/config.py src/soccer_framework/game_state.py src/soccer_framework/ros_truth.py src/soccer_framework/robot.py src/soccer_framework/game_controller.py src/soccer_framework/telemetry.py src/strategy.py
python3 -m unittest tests/test_soccer_framework_core.py tests/test_soccer_framework_runtime.py tests/test_soccer_framework_telemetry.py
```

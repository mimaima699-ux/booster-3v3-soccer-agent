---
name: soccersim-auto-debug
description: Automate SoccerSim 3v3 strategy debugging in this repository. Use when the user asks Codex to run automated SoccerSim matches, deploy to the configured simulator container from ros_debug/.soccer_sim.env or SOCCER_DOCKER_CONTAINER, send ready/set/playing referee control, enable real GameControl Auto Match, inspect ROS topics and telemetry, evaluate kicking quality or referee penalties, iterate on src strategy/runtime code, or compare match reports such as artifacts/auto_debug/*/report.md.
---

# SoccerSim Auto Debug

Use this skill for repository-local SoccerSim strategy iteration. Prefer the real simulator GameControl HTTP service over mock referee messages unless the user explicitly requests pure unit-level strategy testing.

## Scope

- Repository root: `/home/ylb/BoosterStudioProjects/SoccerSim`
- Simulator container: read `SOCCER_DOCKER_CONTAINER` from `ros_debug/.soccer_sim.env` or the environment
- Container workdir: `/root/workspace/SoccerSim`
- Primary automation script: `scripts/auto_debug_match.py`
- Debug ROS package: `ros_debug/soccer_sim_debug`
- Authoritative rules: `note/3v3_rule.md`
- GameControl HTTP URL in the container: `http://127.0.0.1:38383`
- GameControl event artifacts in the container: `/usr/local/booster_robot/booster_robocup_sim/logs/game-control`

Default team mapping:

- team1: `robot1,robot2,robot3`
- team2: `robot4,robot5,robot6`

## Standard Workflow

1. Inspect current worktree before editing:

   ```bash
   git status --short
   ```

2. If changing strategy or runtime behavior, read the relevant code before editing. Common files:

   - `src/soccer_framework/types.py`
   - `src/soccer_framework/config.py`
   - `src/soccer_framework/world.py` / `robot.py` / `game_controller.py`
   - `src/runtime.py`
   - `src/strategy.py`
   - `src/tactics/geometry.py`
   - `src/tactics/targeting/restart.py`
   - `note/3v3_rule.md`

3. Run local focused checks after edits:

   ```bash
   python3 -m py_compile scripts/auto_debug_match.py tests/test_auto_debug_match.py src/__init__.py src/main.py src/runtime.py src/soccer_framework/__init__.py src/soccer_framework/types.py src/soccer_framework/config.py src/soccer_framework/game_state.py src/soccer_framework/world.py src/soccer_framework/robot.py src/soccer_framework/game_controller.py src/soccer_framework/telemetry.py src/strategy.py ros_debug/soccer_sim_debug/soccer_sim_debug/ros_node.py
   python3 -m unittest tests/test_soccer_framework_core.py tests/test_soccer_framework_runtime.py tests/test_soccer_framework_telemetry.py tests/test_auto_debug_match.py
   ```

4. Deploy and build in the configured container. The deployment scripts read `ros_debug/.soccer_sim.env`, so container changes should update that file instead of editing code:

   ```bash
   scripts/debug_build
   ```

5. Run a quick control/observer smoke when validating the automation chain only:

   ```bash
   set -a
   source ./ros_debug/.soccer_sim.env
   set +a
   docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc 'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py --local-container --no-start-nodes --ready-wait-sec 2 --play-sec 3 --observe-hz 2 --min-score 0 --run-id smoke_no_nodes_$(date +%Y%m%d_%H%M%S)'
   ```

6. Prefer real GameControl Auto Match when validating rule compliance or strategy quality:

   ```bash
   set -a
   source ./ros_debug/.soccer_sim.env
   set +a
   docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc 'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py --local-container --auto-match --play-sec 70 --observe-hz 5 --run-id automatch_$(date +%Y%m%d_%H%M%S)'
   ```

   Auto Match sends `resetGame`, enables `POST /auto-match {"enabled": true}`, lets the simulator referee advance Ready / Set / Playing, then disables Auto Match at the end. Use this mode for illegal position, motion in Set, incapable robot, illegal reentry, ball-out, score, and restart evaluation.

7. Run a manual referee-state full match iteration when you specifically need deterministic ready/set/playing timing:

   ```bash
   docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc 'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py --local-container --ready-wait-sec 5 --play-sec 15 --observe-hz 5 --run-id auto_strategy_$(date +%Y%m%d_%H%M%S)'
   ```

   For human visual inspection, use a longer play window:

   ```bash
   docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc 'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py --local-container --ready-wait-sec 5 --play-sec 45 --observe-hz 5 --run-id visual_$(date +%Y%m%d_%H%M%S)'
   ```

8. Copy the report back to the local repo when the run happened inside the container:

   ```bash
   docker cp "$SOCCER_DOCKER_CONTAINER:/root/workspace/SoccerSim/artifacts/auto_debug/<run_id>" artifacts/auto_debug/
   ```

9. Confirm no automation nodes are left running:

   ```bash
   docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc 'ps -eo pid,ppid,stat,cmd | grep -E "auto_debug_match|soccer_auto_debug_observer|soccer_sim_node.*soccer_auto_debug" | grep -v grep || true'
   ```

## Report Interpretation

Open `artifacts/auto_debug/<run_id>/report.md` and `report.json`.

Core success signals:

- `states_seen` includes `READY` and `PLAYING`
- `first_soccer_kick_sec` is not `None`
- `ball_progress_x` is positive for team1 attack direction
- `observer.samples.jsonl` contains only JSON object lines
- `observer.log` is empty or contains only explainable ROS diagnostics
- no `soccer_auto_debug_*` process remains after the run

Auto Match and referee compliance signals:

- `auto_match_enabled_seen` is `True`
- `referee_penalty_count` is `0` for clean strategy windows
- `referee_warning_count` is `0` or explainable
- `referee_penalty_rule_ids`, `referee_penalty_reasons`, and `referee_penalized_actors` identify what rule failed
- `final_player_penalty_count` is `0` at the end of the run
- `game_control/` contains copied `events.jsonl`, `status.json`, and related GameControl artifacts

Typical score interpretation:

- `>= 90`: automation chain and basic kicking quality are working
- `70-89`: chain is working, but strategy quality or telemetry stability needs inspection
- `< 70`: inspect missing findings before changing strategy blindly

Known non-fatal issues:

- `game_controller_stale_count` may appear during transitions; inspect whether stale events persist into PLAYING.
- `warn_or_error_count` may mirror stale telemetry. Treat repeated WARN/ERROR during steady PLAYING as a real issue.
- In Auto Match runs, a low score can be caused by referee penalties even when the ball moved. Inspect `referee_penalty_rule_ids` before changing kick mechanics.

## Strategy Iteration Rules

- Keep edits scoped to strategy/runtime behavior relevant to the report findings.
- Do not use `Any` or broad `cast` to hide type issues.
- Use explicit type narrowing before passing external/JSON/ROS values into typed APIs.
- Follow `note/3v3_rule.md` when rule documents disagree.
- After each strategy edit, run local checks and at least one full automated match.
- Compare the new report to the prior run. Report score alone is not enough; mention kick timing, ball progress, stale/WARN counts, referee penalty counts/reasons, and whether nodes cleaned up.

## Useful User Prompts

The user can ask:

- `使用本项目 skills/soccersim-auto-debug，跑一轮完整自动化比赛并把报告复制回来。`
- `使用本项目 skills/soccersim-auto-debug，用真实 Auto Match 跑一轮，重点看自动裁判违规和处罚原因。`
- `使用本项目 skills/soccersim-auto-debug，调整策略让球推进更稳定；每次改完跑自动化评估，给我对比报告。`
- `使用本项目 skills/soccersim-auto-debug，跑 3 轮 45 秒 visual 测试，我肉眼看效果。`
- `使用本项目 skills/soccersim-auto-debug，根据 artifacts/auto_debug/<run_id>/report.md 分析为什么扣分。`

## Failure Handling

- If `source /opt/ros/humble/setup.bash` fails under `set -u`, use `set -eo pipefail`.
- If host Python cannot access Docker socket, use direct `docker exec` / `docker cp` commands and `--local-container`.
- If `ros_debug` is ignored by git status, still inspect and copy it; it is required by the debug build.
- If a full run leaves processes behind, terminate only automation nodes matching `soccer_sim_node` and `__node:=soccer_auto_debug_team`.

# 操作说明

## 开发协议

踢球数据 topic、裁判机 JSON 字段和当前使用到的 boosteros 控制接口集中整理在 `docs/developer_protocol.md`。

## 日志位置

- 默认结构化日志：`/tmp/booster_agent/soccer_logs/`
- 自动调试产物：`artifacts/auto_debug/<run_id>/`
- 自动调试时容器内 debug node 日志：`/tmp/booster_agent/soccer_auto_debug/<run_id>/`

## 自动调试链路

主入口是：

```bash
python3 scripts/auto_debug_match.py --build
```

脚本会读取本地 `ros_debug/.soccer_sim.env` 里的容器配置，把调试运行所需的最小工作区复制到容器内 `/root/workspace/SoccerSim`，编译 `ros_debug/soccer_sim_debug`，启动两队 ROS2 debug node。部署范围包括 `src`、`ros_debug`、`scripts`、`res` 和必要的顶层配置文件，不会复制 `artifacts`、`.venv`、`build` 等本地大目录：

- 当前本机配置文件：`ros_debug/.soccer_sim.env`（gitignored）
- 模板配置文件：`ros_debug/.soccer_sim.env.example`
- 容器配置优先级：命令行参数 `--container` > 环境变量 `SOCCER_DOCKER_CONTAINER` > `ros_debug/.soccer_sim.env` > 内置模板兜底
- 这组变量（`SOCCER_DOCKER_*` / `SOCCER_FASTDDS_PROFILE` / `SOCCER_GAME_CONTROL_*`）只被 `scripts/` 下的部署脚本读，不进容器内 runtime 进程

- team1: `robot1,robot2,robot3`
- team2: `robot4,robot5,robot6`
- 默认策略：team1/team2 都使用 `advanced`

然后脚本会通过仿真 GameControl HTTP API 执行：

1. `resetGame`
2. `ready`
3. 等待 `--ready-wait-sec`
4. `set`
5. `playing`
6. 等待 `--play-sec`
7. 采集并评分

如果加上 `--auto-match`，脚本会改用真实裁判机自动比赛流程：

1. `resetGame`
2. `POST /auto-match {"enabled": true}`
3. 由裁判机自动进入 Ready / Set / Playing
4. 比赛中由裁判机自动处理出界、进球、重新开球和违规处罚
5. 等待 `--play-sec`
6. 采集 `/status`、`/result`、GameControl `events.jsonl`
7. `POST /auto-match {"enabled": false}`

这个模式不是 mock 裁判消息，适合自动发现非法站位、Set 中移动、无能力机器人、非法回场等违规。

常用短测：

```bash
python3 scripts/auto_debug_match.py \
  --build \
  --ready-wait-sec 5 \
  --play-sec 15 \
  --observe-hz 5
```

真实裁判机 Auto Match 短测：

```bash
python3 scripts/auto_debug_match.py \
  --build \
  --auto-match \
  --play-sec 70 \
  --observe-hz 5
```

只验证裁判控制和 ROS topic 采样，不启动策略节点：

```bash
python3 scripts/auto_debug_match.py \
  --build \
  --no-start-nodes \
  --ready-wait-sec 2 \
  --play-sec 3 \
  --min-score 0
```

如果已经手动启动了 debug node，也可以跳过 build：

```bash
python3 scripts/auto_debug_match.py --no-start-nodes
```

在 Codex/IDE 沙箱里如果 Python 子进程不能访问 Docker socket，可以先直接用现有部署脚本把工程复制并编译到容器，然后在容器内用 `--local-container` 跑自动化链路：

```bash
set -a
source ./ros_debug/.soccer_sim.env
set +a

scripts/debug_build

docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc \
  'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py \
    --local-container \
    --no-start-nodes \
    --ready-wait-sec 2 \
    --play-sec 3 \
    --observe-hz 2 \
    --min-score 0'
```

完整启动两队 debug node 的容器内短测：

```bash
set -a
source ./ros_debug/.soccer_sim.env
set +a

docker exec "$SOCCER_DOCKER_CONTAINER" bash -lc \
  'cd /root/workspace/SoccerSim && python3 scripts/auto_debug_match.py \
    --local-container \
    --auto-match \
    --play-sec 70 \
    --observe-hz 5'
```

`--min-score` 默认是 `40`。做链路烟测时可以设为 `0`，质量门禁或批量迭代时保留默认值或调高。

## 策略 Profile

运行时通过同一个策略接口选择不同实现：

- `advanced`：完整 AI。包含 Ready/Set/Playing 状态处理、重开球避让、动态职责配合、开放前传/带球选择、门将防守和对手位姿绕障。
- `basic`：普通策略。保留合法站位、重开球避让、追球、支援和门将防守，便于做回归基线。
- `template`：示例模板。中路槽位优先追球并踢向进攻方向，侧翼 / keeper 槽位保持简单合法位置，适合参赛者从少量逻辑开始调参和补策略。

环境变量入口：

```bash
SOCCER_ROBOT_NAMES=robot1,robot2,robot3
SOCCER_TEAM_ID=1
```

ROS2 debug node 参数入口：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p team_id:=1 \
  -p robot_names:=robot1,robot2,robot3 \
  -p opponent_robot_names:=robot4,robot5,robot6 \
  -p ready_slots:=center,side,keeper \
  -p strategy_profile:=advanced
```

自动调试对打示例：

```bash
python3 scripts/auto_debug_match.py \
  --build \
  --auto-match \
  --team1-strategy advanced \
  --team2-strategy template \
  --play-sec 70 \
  --observe-hz 5
```

比较普通策略和高阶策略：

```bash
python3 scripts/auto_debug_match.py \
  --build \
  --auto-match \
  --team1-strategy basic \
  --team2-strategy advanced \
  --play-sec 70 \
  --observe-hz 5
```

## Codex 自动迭代指令

本项目有一个项目本地 skill：

```text
skills/soccersim-auto-debug/
```

后续新会话里可以明确这样说：

```text
使用本项目 skills/soccersim-auto-debug，调整策略让球推进更稳定；每次改完跑自动化评估，给我对比报告。
```

常用指令模板：

```text
使用本项目 skills/soccersim-auto-debug，跑一轮完整自动化比赛并把报告复制回来。
```

```text
使用本项目 skills/soccersim-auto-debug，跑 3 轮 45 秒 visual 测试，我肉眼看效果。
```

```text
使用本项目 skills/soccersim-auto-debug，根据 artifacts/auto_debug/<run_id>/report.md 分析为什么扣分，并提出下一步策略改动。
```

```text
使用本项目 skills/soccersim-auto-debug，修改策略后按 ros_debug/.soccer_sim.env 自动部署，跑完整短测；如果分数或球推进变差就回看报告并继续迭代。
```

## 产物

每轮输出目录：

```text
artifacts/auto_debug/<run_id>/
  config.json
  timeline.json
  observer.samples.jsonl
  telemetry/
  game_control/
  report.json
  report.md
  team1.node.log
  team2.node.log
  observer.log
```

`report.md` 会给出本轮分数和主要判断：

- 是否持续收到 `/soccer/game_controller`
- 是否观察到 `READY` / `PLAYING`
- 是否采集到 `/team1/.../ground_truth/ball`
- debug node telemetry 是否出现 `soccer_kick_started`
- 球在 team1 视角下是否向 `+x` 方向推进
- 是否有 `game_controller_stale`
- 是否有 WARN / ERROR telemetry
- `--auto-match` 时是否确认真实 Auto Match 已启用
- 自动裁判 `penalty` / `warning` 事件数量、处罚原因和相关机器人
- 最终 `/status` 中是否仍有场上机器人处于 penalty

## 裁判机控制面

自动脚本不依赖宿主机端口映射，而是把自身复制到容器内后，通过容器内 helper 访问：

```text
http://127.0.0.1:38383
```

等价手动命令：

```bash
set -a
source ./ros_debug/.soccer_sim.env
set +a

docker exec "$SOCCER_DOCKER_CONTAINER" python3 /tmp/soccer_auto_debug_match.py \
  --inside-control \
  --method POST \
  --url http://127.0.0.1:38383/control \
  --payload '{"type":"ready","side":"home"}'
```

如果只想看状态：

```bash
docker exec "$SOCCER_DOCKER_CONTAINER" python3 /tmp/soccer_auto_debug_match.py \
  --inside-control \
  --method GET \
  --url http://127.0.0.1:38383/status
```

如果只想手动触发真实 Auto Match：

```bash
docker exec "$SOCCER_DOCKER_CONTAINER" python3 /tmp/soccer_auto_debug_match.py \
  --inside-control \
  --method POST \
  --url http://127.0.0.1:38383/auto-match \
  --payload '{"enabled":true}'
```

关闭 Auto Match：

```bash
docker exec "$SOCCER_DOCKER_CONTAINER" python3 /tmp/soccer_auto_debug_match.py \
  --inside-control \
  --method POST \
  --url http://127.0.0.1:38383/auto-match \
  --payload '{"enabled":false}'
```

## 注意

- 当前主链路使用仿真程序真实 GameControl service，不使用 mock 裁判 topic。
- mock topic 只适合纯策略分支调试，不能验证摆球、比分、出界、处罚和真实事件。
- 自动裁判事件从容器内 GameControl 结果目录复制到 `artifacts/auto_debug/<run_id>/game_control/`。
- 如果 Codex/IDE 沙箱不允许 Python 子进程访问 Docker socket，优先使用上面的 `--local-container` 方式。

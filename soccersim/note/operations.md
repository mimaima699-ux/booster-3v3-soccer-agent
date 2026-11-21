# SoccerSim 运行手册

本文档记录怎么在容器里跑 SoccerSim：环境变量、ROS 参数、Docker 调试脚本、仿真测试建议和本地验证。架构和策略说明见 [implementation.md](implementation.md)。

## 1. 运行配置

默认配置是 3 个机器人：

```text
robot1 -> player_id=1 -> ReadySlot.CENTER
robot2 -> player_id=2 -> ReadySlot.SIDE
robot3 -> player_id=3 -> ReadySlot.KEEPER
```

`ReadySlot` 只决定 READY、重开球避让和底线恢复等固定站位；PLAYING 阶段由 `AssignRoles` 每帧写入动态职责。

Booster Agent 启动时只通过环境变量注入比赛身份和己方机器人名，不需要改代码：

| 环境变量 | 含义 | 示例 |
| --- | --- | --- |
| `SOCCER_ROBOT_NAMES` | 逗号分隔的机器人名，顺序决定 `player_id`。 | `robot1` 或 `robot1,robot2` |
| `SOCCER_TEAM_ID` | 队伍编号。 | `1` / `2` |

对手机器人名由 `SOCCER_TEAM_ID` 推导，控制频率默认 `30Hz`，裁判状态 topic 固定默认 `/soccer/game_controller`。这些值不再通过 envvar 覆盖；要调整请改 `SoccerConfig` 默认值，或在构造 `SoccerConfig` 时传入定制配置（ROS 调试期可经 ROS 参数注入）。

速度上限、踢球迟滞（`max_linear_speed` / `max_angular_speed` / `soccer_kick_*`）等战术旋钮已收进 `SoccerStrategyTuning`，行为树 tick trace 等调试开关已收进 `SoccerDebugConfig`，不再通过 envvar 覆盖；要调整请改对应 dataclass 默认值，或在构造 `SoccerConfig` 时传入定制配置（ROS 调试期可经 ROS 参数注入）。

场地尺寸固定为 adultsize（不再支持 envvar / ROS 参数覆盖；要换尺寸请改 `SoccerConfig` 默认值或在程序里构造时传参）。默认值参照 `soccer-python`：

| 配置 | 默认值 |
| --- | --- |
| `field_length` / `field_width` | `14.0` / `9.0` |
| `penalty_dist` | `2.1` |
| `goal_width` | `2.6` |
| `center_circle_radius` | `1.5` |
| `penalty_area_length` / `penalty_area_width` | `3.0` / `6.0` |
| `goal_area_length` / `goal_area_width` | `1.0` / `4.0` |
| 队伍场地坐标系球门坐标 | 己方 `x=-7.0` / 对方 `x=7.0` |

## 2. 启动方式

### 2.1 单机器人 / 小规模槽位组合

单机器人用中路槽位测试踢球能力：

```bash
SOCCER_ROBOT_NAMES=robot1
```

单机器人用 keeper 槽位测试门将站位：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=robot1 \
  -p ready_slots:=keeper
```

两机器人 1v1 / 小规模槽位验证：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=robot1,robot2 \
  -p ready_slots:=center,keeper
```

注意：当前 Agent 仍然需要裁判状态和球 / 位姿真值。没有 GameController PLAYING 状态时，策略会按规则停住；没有球或机器人位姿时，chaser 也不会进入踢球逻辑。

### 2.2 ROS2 调试包 `soccer_sim_debug`

ROS2 调试 Node 是独立包 `soccer_sim_debug`，位于 [ros_debug/soccer_sim_debug/](../ros_debug/soccer_sim_debug/)。根目录不再有 `package.xml` / `setup.py` / `setup.cfg`，所以根目录本身不会被 colcon 当成 ROS 包。

```bash
colcon build --base-paths ros_debug --packages-select soccer_sim_debug
source install/setup.bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=robot1,robot2,robot3 \
  -p ready_slots:=center,side,keeper \
  -p opponent_robot_names:=robot4,robot5,robot6 \
  -p team_id:=1
```

在仿真容器内单机器人调试时，可以让 `virtual_robot_name` 走默认机器人连接，也就是传 `robot_names:=default`：

```bash
cd /root/workspace/SoccerSim
source /opt/ros/humble/setup.bash
source install/setup.bash
FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile.xml \
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=default \
  -p ready_slots:=center \
  -p control_hz:=10
```

单机器人中路槽位踢球能力验证：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=robot1 \
  -p ready_slots:=center
```

真实默认机器人可以把名字写成 `default`，runtime 会传空字符串给 `BoosterRobot.virtual_robot_name`：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=default \
  -p ready_slots:=center
```

### 2.3 Docker 编译和启动脚本

```bash
# 默认读取 ros_debug/.soccer_sim.env 中的 SOCCER_DOCKER_CONTAINER
# 默认容器内工程目录：/root/workspace/SoccerSim
scripts/debug_build
scripts/debug_run.sh
```

`scripts/debug_build` 会先把调试运行所需的最小工作区复制到容器内的 `SOCCER_DOCKER_WORKDIR`，再执行 `colcon build`。复制范围包括 `src`、`ros_debug`、`scripts`、`res` 和必要的顶层配置文件，不会复制 `artifacts`、`.venv`、`build` 等本地大目录。

容器和工程目录可以写入本机配置文件，也可以用环境变量临时覆盖：

```bash
cp ros_debug/.soccer_sim.env.example ros_debug/.soccer_sim.env
printf 'SOCCER_DOCKER_CONTAINER=<container-id>\n' > ros_debug/.soccer_sim.env

SOCCER_DOCKER_CONTAINER=<container-id> \
SOCCER_DOCKER_WORKDIR=/root/workspace/SoccerSim \
scripts/debug_build
```

启动脚本不传参数时默认按单机器人中路槽位调试启动：

```bash
ros2 run soccer_sim_debug soccer_sim_node --ros-args \
  -p robot_names:=default \
  -p ready_slots:=center \
  -p control_hz:=10
```

如果要覆盖 ROS 参数，把 `ros2 run` 后面的参数直接传给脚本：

```bash
scripts/debug_run.sh --ros-args \
  -p robot_names:=robot1,robot2,robot3 \
  -p ready_slots:=center,side,keeper \
  -p team_id:=1
```

## 3. 仿真测试建议

启动后优先看这些日志 / 行为：

1. 是否打印 3 组 truth topic 订阅。
2. 是否成功创建 `robot1`、`robot2`、`robot3`，并进入 `robocup` gait / `walk` mode。
3. 是否打印 `Subscribed GameController state topic: /soccer/game_controller`。
4. 裁判状态变化时，`Soccer control state=...` 的 reason 是否符合预期。
5. 如果 `PLAYING` 中一直显示 `center waiting for ball` 或没有 chaser 动作，优先检查球真值消息格式。此时应先改 `RosTruthProvider` 的球解析，而不是改策略判断。
6. 断开裁判 topic 超过 2 秒时，三机应停止。
7. `SET` 状态下三机必须停止，不能继续走位或踢球。

## 4. 本地验证

当前本地已通过：

```bash
python3 -m py_compile src/main.py src/runtime.py src/soccer_framework/__init__.py src/soccer_framework/types.py src/soccer_framework/config.py src/soccer_framework/game_state.py src/soccer_framework/world.py src/soccer_framework/robot.py src/soccer_framework/game_controller.py src/soccer_framework/telemetry.py src/strategy.py ros_debug/soccer_sim_debug/soccer_sim_debug/ros_node.py tests/test_soccer_framework_core.py tests/test_soccer_framework_runtime.py tests/test_soccer_framework_telemetry.py
python3 -m unittest tests/test_soccer_framework_core.py tests/test_soccer_framework_runtime.py tests/test_soccer_framework_telemetry.py
```

`py_compile` 验证语法和导入；`unittest` 覆盖纯策略、规则、运行时适配和结构化 logger/trace 的本地行为。它们不验证专用仿真环境中的 ROS topic 与 boosteros SDK 行为。

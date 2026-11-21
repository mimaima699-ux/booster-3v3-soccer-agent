# 双足机器人行走导航控制设计文档

> 本文档描述 `MotionController` 的设计，作为双足机器人足球领域行走导航的参考实现。
> 目标是沉淀一套可复用的设计，避免每次新项目都从零重写。

## 1. 场景与约束

### 1.1 目标场景

- **双足人形机器人**足球比赛（如 RoboCup SPL、3v3 仿真）
- **小场地**（14m × 9m 量级），障碍物只有几个机器人
- **高实时性**：每帧为多个机器人生成命令（3v3 = 3 个）
- **无全局地图**：场地固定已知，不需要 SLAM/建图

### 1.2 物理约束

双足底盘的 `set_velocity(vx, vy, vyaw)` 有步态合成约束：

- `vx + vyaw` 是稳定组合（前进 + 转向）
- `vy` 横移可单独发，但**不能和 `vx` 同时给**（步态合成会出问题）
- `vyaw` 可单独发（纯转向）

这意味着通用导航库（Nav2、DWA、势场法）的"全向速度分解"不适用——它们假设可同时给 `vx + vy`。双足场景必须用不同的避障策略。

### 1.3 为什么不用第三方库

| 库 | 不适用原因 |
|----|-----------|
| ROS 2 Nav2 | 面向大场地自主导航，需建图/定位/代价地图，严重过载；不处理双足步态约束 |
| PythonRobotics (DWA/势场) | 假设全向/差速底盘，不处理 `vx+vy` 互斥；教科书实现，无工业封装 |
| 通用局部规划器 | 假设 omni 底盘，速度空间采样对双足步态无意义 |

**结论**：双足足球场景需要自研 reactive 控制器。RoboCup SPL 各队也是自研，无通用库可用。

## 2. 架构：Reactive Control（反应式控制）

### 2.1 架构选型

| 架构 | 适用场景 | 我们的选择 |
|------|---------|-----------|
| 全局规划 + 局部规划 (move_base) | 大场地、未知地图、需最优路径 | ❌ 过载 |
| Behavior-based (Subsumption) | 多行为并行仲裁 | ❌ 单一目标场景不需要 |
| **Reactive (反应式)** | 小场地、少障碍、高实时 | ✅ |

Reactive 架构核心：**感知→动作直接映射，无全局路径规划**。每帧根据当前障碍物反应式算绕行点，直接出速度。

### 2.2 三层避障设计

双足约束决定了避障必须分三层，每层改不同的东西：

```
move_to_target(player, world, target, reason)
│
├─ 1. 静态避障（改目标点）
│   输入: target, 障碍列表
│   输出: adjusted_target（可能是 via，可能就是原 target）
│   状态: _avoid_side_by_player（记忆每个球员从哪边绕，防抖动）
│
├─ 2. 行走控制（算速度）
│   输入: adjusted_target, 当前位姿
│   输出: vx + vyaw（vy 强制 0）
│   状态: 无（纯函数）
│
└─ 3. 动态避障（改角速度）
    输入: command(vx, vyaw), 近邻列表
    输出: vx + 调整后的 vyaw
    状态: 无（纯函数）
```

**为什么分三层？** 因为双足不能同时给 `vx + vy`：

- 静态层靠**改目标点**让行走控制"自己绕"（输出 `vx + vyaw`，`vy=0`）
- 动态层靠**加 `vyaw` 偏置**让机器人"多转一点"绕开近邻（也不碰 `vy`）

三层改的东西不冲突，物理约束得以满足。

## 3. 核心类：`MotionController`

### 3.1 职责

把"想去哪"翻译成"怎么走/怎么踢"的 `RobotCommand`。避障 + 行走控制 + 踢球一体化。

### 3.2 依赖

```python
class MotionController:
    def __init__(self, config, field, kicker):
        self._config = config          # 速度上限、策略参数
        self._field = field            # 场地几何（clamp_inside_field 等）
        self._kicker = kicker          # 踢球迟滞状态机
        self._obstacles = ObstacleCollector(config, field)  # 无状态障碍收集
        self._avoid_side_by_player: dict[int, float] = {}   # via 侧记忆
```

### 3.3 对外接口

| 方法 | 职责 |
|------|------|
| `move_to_target` | 移动命令（含三层避障），返回 `RobotCommand` |
| `kick_command` | 踢球命令，标记踢球迟滞 |
| `approach_target` | 球后方助跑目标点计算 |

### 3.4 `move_to_target` 完整流程

```python
def move_to_target(self, player_id, world, target, reason,
                   arrive_distance=None, hold_vyaw=0.0,
                   avoid_opponents_dynamically=False) -> RobotCommand:
    robot = world.robots.get(player_id)
    if robot is None or robot.pose is None:
        return RobotCommand.stop(f"{reason}: waiting for pose")

    # 1. 静态避障：算 via 点
    adjusted_target = self._avoidance_target(player_id, robot.pose, target, world)
    adjusted_reason = f"{reason} via obstacle" if adjusted_target != target else reason

    # 2. 行走控制：算 vx + vyaw
    command = self._compute_velocity(robot.pose, adjusted_target, adjusted_reason, ...)

    # 3. 动态避障：叠加 vyaw 偏置
    return self._apply_dynamic_avoidance(player_id, world, command, ...)
```

## 4. 各层详解

### 4.1 静态避障层

**目标**：在原始路径上插入绕行 via 点。

**算法**（反应式 via-point）：

1. 沿 `start → target` 线段找第一个挡路障碍
2. 在障碍侧面（左或右）生成 via 点作为新目标
3. via 侧选择有跨帧记忆，避免左右抖动

**挡路判定**（路径局部坐标系投影）：

- `along`（沿路径投影）必须落在路径中段：起点/终点附近的障碍不算挡路
- `lateral`（垂直路径投影）必须小于 `障碍半径 + 安全余量`，即障碍侵入了走廊

**via 点生成**：

```
障碍投影到路径 → closest 点
沿路径左侧法线偏移 (radius + margin) → via 点
via 朝向指向原 target → 让行走控制自然拐回
```

**via 侧记忆**：`_avoid_side_by_player[player_id]` 记住每个球员上次从哪边绕。首次遇到障碍时按几何最短选择（障碍在左→从右绕），之后帧保持同侧，避免障碍中心抖动时 via 点左右跳变。

### 4.2 行走控制层

**目标**：unicycle 风格的速度计算。

**输出约束**：永远满足以下其一
- 纯停止：`vx=vy=vyaw=0`
- 纯转向：`vyaw != 0, vx=vy=0`
- 前进+转向：`vx > 0, vy=0, vyaw 小`

**控制分支**：

```
距离 < arrive_distance 且朝向对齐 → 停止（或 hold_vyaw 自转）
距离 < arrive_distance 但朝向未对齐 → 纯转向对齐
角度误差 > turn_threshold → 纯转向
否则 → 前进 + 跟随转向
```

**速度公式**：

- `vx = gain * distance * cos(angle_error)`，clamp 到 `[floor, max]`
  - `cos` 项让大角度差时自动减速
  - `floor` 保底让远距离也保持步速
- `vyaw = clamp(2 * angle_error, ±max)`，叠加 `floor` 保底

**参数统一**：PLAY/READY 不再区分，用一套折中参数。差异只剩 `avoid_opponents_dynamically` 一个布尔。

### 4.3 动态避障层

**目标**：近邻威胁映射成 `vyaw` 偏置。

**为什么不和静态层合并？** 静态层处理"路径上的障碍"（要绕大弯），动态层处理"近处的人"（要微调转向）。两者威胁尺度不同，合并会让简单场景也触发不必要的绕行。

**威胁强度计算**（同时考虑当前距离和预测最近距离）：

```
当前距离 < min_distance → 按距离比例给强度
预测未来 horizon 秒内的最近距离 < min_distance → 按预测比例给强度
两者取 max
```

**偏置方向**：近邻在左侧 → 朝右转（`yaw_bias < 0`）；近邻在右侧 → 朝左转。正后方按 `player_id` 奇偶决断（避免全队同向挤压）。

**对手开关**：`avoid_opponents_dynamically` 控制。PLAY 阶段关（追球必然贴近对手），READY 阶段开（对手静止挡路要绕）。

## 5. 障碍物收集：`ObstacleCollector`

无状态工具，三类障碍：

| 类型 | 来源 | 半径 |
|------|------|------|
| 对手 | `world.opponents` | `obstacle_radius`（较大） |
| 队友 | `world.robots`（排除自己） | `teammate_obstacle_radius`（较小） |
| 球门结构 | 4 门柱 + 3 面网采样 | `post_radius` / `net_radius` |

**过滤**：`pose` 为空或 `last_seen_at` 超过 2 秒的跳过（位置不可信）。

**球门采样**：U 形结构（4 门柱 + 后网 + 左右网）均匀撒圆形障碍，采样间距 < 走廊检测半径，保证任何角度切入都击中至少一个采样点。

## 6. 设计要点总结

### 6.1 为什么是 reactive 而非 behavior-based

- **单一目标场景**：每帧只有一个目标点，不需要多行为并行仲裁
- **小场地少障碍**：不需要全局规划
- **高实时性**：reactive 每帧 O(n) 障碍扫描，足够快

### 6.2 为什么三层分层

双足 `vx + vy` 互斥约束决定的：

- 静态层改目标点（不碰速度）
- 行走层算 `vx + vyaw`（`vy=0`）
- 动态层改 `vyaw`（不碰 `vx`/`vy`）

三层改的东西正交，互不干扰。

### 6.3 为什么 via 侧有跨帧记忆

障碍中心位置每帧会抖动（感知噪声）。如果每帧重新选侧，via 点会在左右两侧跳变，机器人走 Z 字形。记忆上次选择的侧，保持连续性，直到障碍消失或路径被新障碍挡死才切换。

### 6.4 为什么 PLAY/READY 不区分行走参数

原设计有两个 profile，但差异大多是调参时的随手值，不是物理约束：

- `arrive_distance`：0.12 vs 0.18 → 统一 0.15
- `turn_threshold`：0.8 vs 0.5 → 统一 0.5
- `floor`：0 vs 0.4 → 统一 0.4（追球时角度误差小不触发）

真正必要的差异只有 `avoid_opponents_dynamically`（PLAY 关、READY 开），这是避障层的事，不是行走控制的事。

## 7. 可复用性

这套设计可开源为双足机器人足球的行走导航库，关键接口：

```python
class MotionController:
    def move_to_target(self, player_id, world, target, reason,
                       arrive_distance=None, hold_vyaw=0.0,
                       avoid_opponents_dynamically=False) -> RobotCommand: ...
    def kick_command(self, player_id, world, ball, kick_theta, reason) -> RobotCommand: ...
    def approach_target(self, ball, kick_theta, approach_offset=0.4) -> Pose2D: ...
```

**适配新平台只需**：
- 实现 `WorldState`（提供机器人位姿、对手位姿、球位姿）
- 实现 `RobotCommand`（`MoveIntent(vx, vy, vyaw)` + `KickIntent`）
- 配置 `SoccerConfig`（速度上限、避障参数）

**不依赖**：ROS、具体机器人 SDK、建图/定位/全局规划。

## 8. 参考与进一步阅读

- RoboCup SPL 各队实现（自研 reactive，无通用库）
- Brooks, R. A. "A robust layered control system for a mobile robot" (Subsumption 架构，我们没用但值得了解)
- Arkin, R. C. "Motor Schema-Based Mobile Robot Navigation" (行为式架构参考)
- ROS 2 Nav2 (对比：为什么通用库不适合双足足球)
- PythonRobotics (DWA/势场法参考实现，思路可借鉴但需适配双足约束)

# 机器人进入球门后撞球网走不出来问题分析

**日期**: 2026-06-26  
**问题**: 机器人面向球门走进球门后，会一直撞球网，走不出来

## 问题描述

机器人在追球射门或门将扑救时，进入球门内部后无法脱困，表现为：
- 持续撞击球网
- 原地打转
- 无法返回场内

## 根本原因

### 1. 球门结构特性

球门是**凹形陷阱结构**：
```
        场内
         ↓
    [============]  ← 底线 (x = 4.5m)
    |            |
    | 球门内部空间 |  ← 深度 0.6m
    |            |
    [============]  ← 后网 (x = 5.1m)
     ← 左网  右网 →
```

球门参数（成人场）：
- 宽度: 2.6m
- 深度: 0.6m
- 位置: x ∈ [4.5, 5.1], y ∈ [-1.3, 1.3]

### 2. 当前避障逻辑的局限

**navigation.py 的球门建模**：
```python
def goal_structure_obstacles(self) -> tuple[Obstacle, ...]:
    """把球门当成 U 形结构，撒成一串圆形障碍点"""
    # 4 根门柱 + 3 面网（左、右、后）均匀采样
```

**motion.py 的避障算法**：
```python
# 三层架构
1. 路径绕行：找第一个挡路障碍 → 在侧面算 via 点
2. 行走控制：unicycle 速度 (vx + vyaw)
3. 转向避让：vyaw 偏置
```

**关键缺陷**：
- ❌ 假设机器人在障碍物**外部**，障碍物在路径**上**
- ❌ 没有处理"机器人已经在障碍物**内部**"的情况
- ❌ 球门内部四面都是障碍物，所有方向的 via 点计算都会失败

### 3. 为什么会进入球门

| 场景 | 触发条件 | 说明 |
|------|---------|------|
| 追球射门 | 球进入球门，机器人继续追 | 最常见 |
| 门将扑救 | 危险球在球门附近，门将冲出 | 门将角色 |
| 避障失误 | via 点计算恰好在球门内 | 罕见但可能 |
| 惯性冲入 | 高速接近球门线，刹不住 | 控制问题 |

### 4. 卡住的机制

```
机器人在球门内 (x=4.8, y=0)
  ↓
目标点在场内 (x=2.0, y=0) [经过 clamp_inside_field]
  ↓
计算路径: (4.8, 0) → (2.0, 0)
  ↓
避障检测: 发现路径上有障碍物（前门柱、球网）
  ↓
计算 via 点: 尝试在障碍物侧面生成绕行点
  ↓
问题: via 点也在球门内部，仍然被其他障碍物包围
  ↓
结果: 反复计算 via 点，机器人在球门内打转/撞网
```

## 现有脱困机制

| 机制 | 触发条件 | 作用范围 | 是否解决球门问题 |
|------|---------|---------|----------------|
| `sideline_recovery` | 球贴近边线 | 边线 | ✗ 不触发 |
| `ball_beyond_goal_line` | 球越过球门线 | 球的位置 | ✗ 只检测球，不检测机器人 |
| `clamp_inside_field` | 目标点超出场地 | 目标点 | ✗ 无法帮助已在球门内的机器人 |

**结论**: 缺少"机器人自身在球门内"的检测和脱困逻辑。

## 解决方案

### 方案1：机器人位置检测 + 忽略球门障碍（推荐）

在 `MotionController.move_to_target` 中：
1. 检测机器人是否在球门内部
2. 如果在球门内，从障碍物列表中**排除球门结构**
3. 让机器人直接往目标走，不被球网阻挡

**优点**：
- 最小侵入，只改 motion.py
- 逻辑清晰：在球门内就无视球网
- 适用于所有进球门的场景

**实现**：
```python
# motion.py
def move_to_target(self, player_id, world, target, reason, ...):
    robot = world.teammates.get(player_id)
    if robot is None or robot.pose is None:
        return RobotCommand.stop(...)
    
    # 检测是否在球门内
    in_goal = self._is_robot_in_goal_area(robot.pose)
    
    # 路径绕行：如果在球门内，不考虑球门障碍物
    adjusted_target = self._avoidance_target(
        player_id, robot.pose, target, world,
        exclude_goals=in_goal  # 新参数
    )
    ...
```

### 方案2：特殊脱困目标点

在角色决策层（`default_roles.py`）：
1. 检测机器人是否在球门内
2. 如果在，覆盖当前目标为"球门前方安全区域"
3. 一旦脱困，恢复正常行为

**优点**：
- 脱困意图明确
- 可以选择最优脱困路径

**缺点**：
- 需要在多个角色中添加检测逻辑
- 侵入性较大

### 方案3：增加"后退"模式

当检测到被包围时（四周都是障碍物）：
1. 反转运动方向，后退
2. 后退到空旷区域后恢复正常

**优点**：
- 通用性强，不只解决球门问题

**缺点**：
- 复杂度高
- 可能与双足步态冲突（后退不稳定）

## 推荐实施方案

**方案1** - 在球门内时忽略球门障碍

### 实施步骤

1. **添加球门内检测** (`tactics/geometry.py`)
```python
def is_in_goal_area(config: SoccerConfig, x: float, y: float) -> bool:
    """检测位置是否在任一球门内部"""
    half_length = config.field_length / 2.0
    half_goal_width = config.goal_width / 2.0
    goal_depth = 0.6
    
    # 对方球门
    if x > half_length and x < half_length + goal_depth:
        if abs(y) < half_goal_width:
            return True
    
    # 我方球门
    if x < -half_length and x > -(half_length + goal_depth):
        if abs(y) < half_goal_width:
            return True
    
    return False
```

2. **修改障碍物收集** (`tactics/navigation.py`)
```python
def collect_all(
    self,
    player_id: int,
    world: WorldState,
    exclude_goals: bool = False,  # 新参数
) -> tuple[Obstacle, ...]:
    """汇总障碍物，可选排除球门结构"""
    obstacles = (
        self.opponent_obstacles(world)
        + self.teammate_obstacles(player_id, world)
    )
    
    if not exclude_goals:
        obstacles = obstacles + self.goal_structure_obstacles()
    
    return obstacles
```

3. **更新运动控制** (`tactics/motion.py`)
```python
def move_to_target(self, player_id, world, target, reason, ...):
    robot = world.teammates.get(player_id)
    if robot is None or robot.pose is None:
        return RobotCommand.stop(...)
    
    # 检测是否在球门内
    in_goal = is_in_goal_area(self._config, robot.pose.x, robot.pose.y)
    
    # 如果在球门内，忽略球门障碍物
    adjusted_target = self._avoidance_target(
        player_id, robot.pose, target, world,
        exclude_goals=in_goal
    )
    ...
```

### 预期效果

- ✅ 机器人进入球门后能直接往场内目标走
- ✅ 不会被球网阻挡
- ✅ 自动脱困，无需特殊行为
- ✅ 对正常避障逻辑无影响

### 边界情况处理

1. **在球门口（刚好在门柱线上）**
   - 按"在球门外"处理，保留球门障碍物
   - 避免机器人正常接近球门时误触发

2. **球也在球门内**
   - 仍然追球，因为球门障碍被忽略
   - 裁判会判定进球或球出界，游戏状态会改变

3. **对方机器人也在球门内**
   - 对方机器人仍然是障碍物（不排除）
   - 避免碰撞

## 测试验证

需要验证的场景：
1. ✅ 机器人在对方球门内 → 能走出来
2. ✅ 机器人在己方球门内 → 能走出来
3. ✅ 机器人在场内正常接近球门 → 仍被球门阻挡（不误触发）
4. ✅ 门将在球门附近扑救 → 不受影响

## 相关代码位置

- 障碍物收集：`src/tactics/navigation.py`
- 运动控制：`src/tactics/motion.py`
- 几何工具：`src/tactics/geometry.py`
- 配置参数：`src/soccer_framework/config.py`

## 后续改进

如果方案1效果不理想，可以考虑：
1. 添加明确的"脱困模式"标志，记录机器人正在脱困
2. 脱困时设置更保守的速度限制
3. 添加遥测日志，记录进出球门的频率，监控异常

# 机器人 Ready 状态原地多转几圈问题分析

## 问题描述

机器人在比赛 Ready 状态上场后，会在目标站位附近原地多转几圈才能停下来。

## 运动学约束（来自 motion_controller_design.md）

双足机器人运动学模型特性：
1. **单独控制有效**：单独设置 vx、vy 或 vyaw 可以正常工作
2. **组合使用受限**：同时设置 vx、vy、vyaw 时，vy 会被忽略，表现得像轮式机器人
3. **最佳实践**：组合速度控制只应该使用 `vx + vyaw`，避免使用 vy

当前代码已经遵循了这个约束（motion.py 设计文档明确说明只用 vx + vyaw）。

## 根本原因分析

### 问题1：角速度 floor 值过大

**位置**：`src/tactics/motion.py:48`
```python
_ANGULAR_SPEED_FLOOR = 0.4  # rad/s
```

**问题**：
- 当角度误差在 0.12 ~ 0.20 rad (6.9° ~ 11.5°) 时
- 比例控制计算：`omega = 2.0 * angle_error`，结果为 0.24 ~ 0.40 rad/s
- 如果 `omega < 0.4`，会被强制提升到 0.4 rad/s
- 在控制频率 30Hz 下，每帧转动 0.4/30 = 0.0133 rad (0.76°)
- 角度误差 0.12 rad 时，理应输出 0.24 rad/s，但被放大到 0.4 rad/s（提升 67%）

**后果**：
- 小角度误差被过度放大
- 容易在目标角度附近过冲
- 过冲后角度误差反向，再次施加 floor 转速，形成震荡

### 问题2：到达角度阈值过严格

**位置**：`src/tactics/motion.py:46`
```python
_ARRIVE_ANGLE = 0.12  # rad，约 6.9°
```

**问题**：
- 当距离 < 0.15m 且角度误差 > 0.12 rad 时，进入「纯转向模式」
- 6.9° 的阈值太严格，机器人会不断尝试微调角度
- 容易在 0.12 rad 临界值附近反复横跳：
  - 角度误差 0.13 rad → 纯转向
  - 转过头变成 0.09 rad → 停止
  - 由于惯性或扰动又变成 0.13 rad → 再次纯转向

### 问题3：转向避让叠加导致失控

**位置**：`src/tactics/motion.py:374-452`（`_apply_yaw_avoidance` 方法）

**Ready 状态特殊性**：
- `avoid_opponents=True`（见 `behavior_tree/nodes/actions.py:146`）
- 队友和对手都会触发转向避让

**叠加效应**：
```
基础 vyaw:    0.4 rad/s (floor)
+ 避让偏置:   0.6 rad/s (单个近邻最大，config.py:105)
= 总 vyaw:    1.0 rad/s (达到上限)
```

**后果**：
- 在 30Hz 下，每帧转动 1.0/30 = 0.0333 rad (1.9°)
- 0.5秒内可以转动 0.5 rad (28.6°)
- 如果附近有多个近邻，叠加效应更严重
- 机器人会以最大角速度快速旋转，容易多转几圈

### 问题4：缺少小角度死区

**现状**：
- 只要角度误差 > 1e-6，就会施加至少 floor = 0.4 rad/s 的转速
- 没有「足够接近就停止」的死区机制

**理想行为**：
- 角度误差 < 某个阈值（比如 0.10 rad）时，不施加 floor
- 使用纯比例控制缓慢收敛
- 或者直接停止（容忍小角度误差）

## 综合场景分析

### 典型失控循环

```
1. 机器人接近 Ready 站位
   距离 0.10m，角度误差 0.13 rad (7.4°)

2. 触发纯转向模式（距离 < 0.15m，角度 > 0.12 rad）
   vyaw = 0.4 rad/s (floor)

3. 附近有队友，叠加避让偏置
   vyaw = 0.4 + 0.6 = 1.0 rad/s (达到上限)

4. 快速旋转，0.13 秒后角度误差变为 0
   但由于惯性和控制延迟，继续转动

5. 过冲，角度误差变为 -0.13 rad
   再次触发纯转向（反方向）

6. 重复步骤 2-5，形成震荡
   多次过冲累积 → 多转几圈
```

### 参数敏感性

| 参数 | 当前值 | 影响 |
|------|--------|------|
| `_ANGULAR_SPEED_FLOOR` | 0.4 rad/s | 小角度误差被放大 67%~100% |
| `_ARRIVE_ANGLE` | 0.12 rad (6.9°) | 容易触发反复微调 |
| `yaw_avoid_bias_max` | 0.6 rad/s | 叠加后达到最大角速度 |
| `control_hz` | 30 Hz | 每帧 33ms，惯性影响明显 |

## 解决方案建议

### 方案1：降低 floor + 放宽阈值 + 增加死区（推荐）

**优点**：最小侵入性，保持现有架构
**实施**：

```python
# src/tactics/motion.py:41-50
_ARRIVE_DISTANCE = 0.15
_ARRIVE_ANGLE = 0.20          # 放宽到 11.5°，避免临界抖动
_TURN_THRESHOLD = 0.5
_ANGULAR_SPEED_FLOOR = 0.25   # 降低到 0.25，减少过冲
_ANGULAR_DEAD_ZONE = 0.15     # 新增：< 0.15 rad 不施加 floor
_LINEAR_SPEED_FLOOR = 0.3
_LINEAR_GAIN = 0.9
```

修改 `_angular_velocity` 方法：
```python
def _angular_velocity(self, angle_error: float) -> float:
    """omega = clamp(2 * err, ±max)，小角度不施加 floor。"""
    if abs(angle_error) <= 1e-6:
        return 0.0
    omega = clamp(
        2.0 * angle_error,
        -self._config.strategy.max_angular_speed,
        self._config.strategy.max_angular_speed,
    )
    # 小角度死区：< _ANGULAR_DEAD_ZONE 不施加 floor，纯比例控制
    if abs(angle_error) >= _ANGULAR_DEAD_ZONE:
        floor = min(_ANGULAR_SPEED_FLOOR, self._config.strategy.max_angular_speed)
        if abs(omega) < floor:
            return floor if omega > 0.0 else -floor
    return omega
```

**预期效果**：
- 角度误差 0.13 rad → omega = 0.26 rad/s（不再被提升到 0.4）
- 角度误差 0.10 rad → omega = 0.20 rad/s（纯比例，不施加 floor）
- 到达判定放宽到 11.5°，减少临界抖动

### 方案2：Ready 状态降低转向避让强度

**位置**：`src/soccer_framework/config.py:105-107`

**当前**：
```python
yaw_avoid_bias_max: float = 0.6  # 单个近邻产生的最大 vyaw 偏置
```

**建议**：
- 降低到 0.3 rad/s，避免叠加后达到上限
- 或者在 Ready 状态特殊处理，传入一个缩减系数

**风险**：
- 可能影响 Ready 状态的避障效果
- 需要测试是否会导致碰撞

### 方案3：Ready 状态使用更大的到达距离

**位置**：`behavior_tree/nodes/actions.py:144-147`

**当前**：
```python
return self._kit.motion.move_to_target(
    self._player_id, world, target, f"ready {slot.value}",
    avoid_opponents=True,
)
```

**建议**：
```python
return self._kit.motion.move_to_target(
    self._player_id, world, target, f"ready {slot.value}",
    arrive_distance=0.25,  # 放宽到 0.25m，提前停止
    avoid_opponents=True,
)
```

**优点**：
- Ready 状态不需要精确对位，可以早点停止
- 减少在目标点附近的精细调整

### 方案4：添加角速度衰减

为到达阶段添加渐进式衰减，而不是突然从 floor 跳变到 0：

```python
def _angular_velocity(self, angle_error: float) -> float:
    if abs(angle_error) <= 1e-6:
        return 0.0
    
    omega_raw = 2.0 * angle_error
    omega = clamp(omega_raw, -self._config.strategy.max_angular_speed, 
                  self._config.strategy.max_angular_speed)
    
    # 衰减区：在 [dead_zone, floor_threshold] 之间线性插值
    floor_threshold = _ANGULAR_SPEED_FLOOR / 2.0  # 0.2 rad
    if abs(angle_error) < _ANGULAR_DEAD_ZONE:
        # 纯比例控制
        return omega
    elif abs(angle_error) < floor_threshold:
        # 线性插值：从 dead_zone 的 omega 到 floor_threshold 的 floor
        ratio = (abs(angle_error) - _ANGULAR_DEAD_ZONE) / (floor_threshold - _ANGULAR_DEAD_ZONE)
        floor_val = _ANGULAR_SPEED_FLOOR * ratio
        if abs(omega) < floor_val:
            return floor_val if omega > 0.0 else -floor_val
    else:
        # 正常 floor
        floor = min(_ANGULAR_SPEED_FLOOR, self._config.strategy.max_angular_speed)
        if abs(omega) < floor:
            return floor if omega > 0.0 else -floor
    return omega
```

## 推荐实施顺序

1. **先实施方案1**（降低 floor + 放宽阈值 + 增加死区）
   - 这是最直接的修复，影响面最小
   
2. **测试效果后考虑方案3**（Ready 状态放宽到达距离）
   - 如果方案1还不够稳定，再放宽到达距离
   
3. **根据实际场景调整方案2**（降低转向避让强度）
   - 如果发现避让过于激进，再调整这个参数

4. **方案4作为长期优化**
   - 如果希望更平滑的控制曲线，可以考虑实施

## 验证建议

修改后需要验证的场景：
1. ✅ Ready 状态机器人是否能稳定停在目标位置
2. ✅ Ready 状态附近有队友/对手时是否仍然稳定
3. ✅ PLAY 状态追球是否受影响（应该不受影响，因为追球距离远，不触发 align 分支）
4. ✅ 精确对位场景（比如踢球助跑）是否仍然准确

## 相关代码位置

- 运动控制参数：`src/tactics/motion.py:41-50`
- 角速度计算：`src/tactics/motion.py:359-371`
- 速度计算主逻辑：`src/tactics/motion.py:296-346`
- 转向避让：`src/tactics/motion.py:374-452`
- Ready 动作叶：`src/behavior_tree/nodes/actions.py:126-147`
- 配置参数：`src/soccer_framework/config.py:58-59, 105-107`

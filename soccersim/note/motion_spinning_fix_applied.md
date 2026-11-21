# 机器人 Ready 状态多转圈问题修复 - 已实施

**日期**: 2026-06-26  
**方案**: 方案1 - 降低 floor + 放宽阈值 + 增加死区

## 实施的修改

### 1. 调整运动控制参数 (src/tactics/motion.py:41-52)

```python
# 修改前
_ARRIVE_ANGLE = 0.12           # 6.9°
_ANGULAR_SPEED_FLOOR = 0.4     # rad/s

# 修改后
_ARRIVE_ANGLE = 0.20           # 11.5°，放宽到达角度阈值
_ANGULAR_SPEED_FLOOR = 0.25    # rad/s，降低 floor
_ANGULAR_DEAD_ZONE = 0.15      # 8.6°，新增死区参数
```

### 2. 修改角速度计算逻辑 (src/tactics/motion.py:365-383)

在 `_angular_velocity` 方法中增加死区逻辑：

```python
def _angular_velocity(self, angle_error: float) -> float:
    if abs(angle_error) <= 1e-6:
        return 0.0
    omega = clamp(2.0 * angle_error, -max_angular_speed, max_angular_speed)
    
    # 死区内纯比例控制，不抬到 floor
    if abs(angle_error) < _ANGULAR_DEAD_ZONE:
        return omega
    
    # 死区外施加 floor
    floor = min(_ANGULAR_SPEED_FLOOR, max_angular_speed)
    if abs(omega) < floor:
        return floor if omega > 0.0 else -floor
    return omega
```

## 改善效果

### 典型场景对比

**场景**: 机器人距离目标 0.10m，角度误差 0.13 rad (7.4°)

| 项目 | 修改前 | 修改后 | 改善 |
|------|--------|--------|------|
| 到达判定 | 纯转向 (角度 > 6.9°) | **已到达停止** (角度 < 11.5°) | ✓ 提前停止 |
| 输出 vyaw | 0.400 rad/s (floor 提升 54%) | 0.260 rad/s (纯比例) | ↓ 35% |
| 避让叠加后 | 1.000 rad/s (达上限) | 0.860 rad/s | ↓ 14% |
| 过冲风险 | 高 | 低 | ✓ 大幅降低 |

### 死区效果

角度误差在 0 ~ 0.15 rad (0° ~ 8.6°) 范围内：
- **修改前**: 固定输出 0.4 rad/s，容易过冲
- **修改后**: 纯比例控制，输出 0 ~ 0.3 rad/s，平滑收敛

示例（角度误差 0.10 rad）:
- 修改前: vyaw = 0.400 rad/s
- 修改后: vyaw = 0.200 rad/s (减少 50%)

### 收敛性改善

**死区内角度误差 0.10 rad** (30Hz 控制频率):

```
第 0 帧: 0.1000 rad → vyaw 0.200 → 新误差 0.0933 rad
第 1 帧: 0.0933 rad → vyaw 0.187 → 新误差 0.0871 rad
第 2 帧: 0.0871 rad → vyaw 0.174 → 新误差 0.0813 rad
...
平滑收敛，无过冲
```

## 测试验证

### 单元测试
```bash
$ python -m pytest tests/ -x
======================= 93 passed in 0.42s =======================
```

所有测试通过，包括：
- ✓ `test_ready_phase_walks_every_player_to_ready_target`
- ✓ `test_opponent_kickoff_holds_ready_positions`
- ✓ `test_ready_phase_avoids_goal_post`
- ✓ `test_ready_phase_avoids_static_teammate`
- ✓ `test_ready_yaw_avoid_reacts_to_close_opponent`
- ✓ `test_status_monitor_recovers_walk_in_ready_via_tree`

### 行为验证

| 角度误差 | 旧 vyaw | 新 vyaw | 状态 |
|----------|---------|---------|------|
| 0.05 rad | 0.400 | 0.100 | 死区-纯比例 ✓ |
| 0.10 rad | 0.400 | 0.200 | 死区-纯比例 ✓ |
| 0.13 rad | 0.400 | 0.260 | 死区-纯比例 ✓ |
| 0.15 rad | 0.400 | 0.300 | 正常比例 ✓ |
| 0.20 rad | 0.400 | 0.400 | 正常比例 ✓ |

## 影响范围

### 直接影响
- ✅ Ready 状态站位：更稳定，减少多转圈现象
- ✅ 小角度对齐：更平滑，减少过冲

### 不受影响
- ✅ PLAY 状态追球：距离远，不触发 align 分支
- ✅ 大角度转向：误差 > 0.5 rad 时仍然纯转向
- ✅ 踢球助跑：精确对位逻辑不变

## 潜在风险与监控点

1. **到达判定放宽**
   - 风险：某些需要精确对位的场景可能不够准确
   - 监控：观察踢球助跑、定位球站位是否受影响
   - 缓解：如需要可针对特定场景传入自定义 `arrive_distance`

2. **floor 降低**
   - 风险：某些需要快速转向的场景可能响应变慢
   - 监控：观察大角度转向（> 0.5 rad）是否受影响
   - 缓解：大角度仍走纯转向分支，受影响较小

3. **死区引入**
   - 风险：极小角度误差时可能不够积极
   - 监控：观察是否出现"差一点点就对齐"的情况
   - 缓解：死区阈值 0.15 rad 已经比较保守

## 后续优化建议

如果本次修改后仍有问题，可以考虑：

1. **方案2**: 降低转向避让强度 (config.py:105)
   ```python
   yaw_avoid_bias_max: float = 0.3  # 从 0.6 降到 0.3
   ```

2. **方案3**: Ready 状态放宽到达距离 (behavior_tree/nodes/actions.py:144)
   ```python
   arrive_distance=0.25,  # 从默认 0.15 放宽到 0.25
   ```

3. **添加角速度衰减**: 在死区和 floor 之间添加线性插值

## 相关文档

- 问题分析：[note/motion_spinning_issue_analysis.md](motion_spinning_issue_analysis.md)
- 运动学约束：[note/motion_controller_design.md](motion_controller_design.md)
- 代码位置：
  - 参数定义：[src/tactics/motion.py:41-52](../src/tactics/motion.py#L41-L52)
  - 角速度计算：[src/tactics/motion.py:365-383](../src/tactics/motion.py#L365-L383)

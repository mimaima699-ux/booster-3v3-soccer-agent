---
name: soccersim-motion
description: Use when editing or reviewing SoccerSim motion control, velocity calculation, obstacle avoidance, robot kinematics constraints, walking parameters, or tests related to robot movement in this repository.
---

# SoccerSim Motion Control

Use this skill before changing code that affects robot movement, including:
- Walking velocity calculation and control parameters
- Obstacle avoidance (path planning, via points, yaw avoidance)
- Robot kinematics constraints and hardware interface
- Movement-related behavior tree nodes
- Motion control tests and debugging

## Authority

- Repository root: `/home/ylb/BoosterStudioProjects/SoccerSim`
- Kinematics documentation: `note/motion_controller_design.md`
- Issue analysis reference: `note/motion_spinning_issue_analysis.md`
- Applied fixes: `note/motion_spinning_fix_applied.md`

## Critical: Robot Kinematics Constraints

**This biped robot has special velocity control characteristics**:

1. **Single-axis control works normally**:
   - `set_velocity(vx=1.0, vy=0, vyaw=0)` → forward motion ✓
   - `set_velocity(vx=0, vy=1.0, vyaw=0)` → lateral strafe ✓
   - `set_velocity(vx=0, vy=0, vyaw=1.0)` → rotation ✓

2. **Combined control is limited** (behaves like wheeled robot):
   - `set_velocity(vx=1.0, vy=1.0, vyaw=0)` → vy is **ignored**, only moves forward
   - `set_velocity(vx=1.0, vy=1.0, vyaw=1.0)` → vy is **ignored**, forward + rotation only
   
3. **Best practice**: Use `vx + vyaw` only, keep `vy=0` in combined commands
   - Current codebase already follows this (see `motion.py` design comments)
   - The motion controller outputs `MoveIntent(vx=..., vy=0.0, vyaw=...)`

**Why**: The underlying gait synthesizer can generate lateral steps OR forward steps with turning, but not arbitrary 3-DOF holonomic motion like an omni-wheel robot.

## Motion Control Architecture

### Core Components

1. **MotionController** (`src/tactics/motion.py`)
   - Reactive control: no global path planning
   - Three-layer architecture:
     - **Path detour**: computes via points to avoid blocking obstacles
     - **Walking control**: unicycle-style velocity (vx + vyaw)
     - **Yaw avoidance**: vyaw bias to dodge nearby teammates/opponents
   - All three layers avoid touching `vy` to satisfy biped constraints

2. **ObstacleCollector** (`src/tactics/navigation.py`)
   - Collects teammates, opponents, and goal structure as obstacles
   - Each obstacle has a radius and safety margin

3. **RobotClient** (`src/soccer_framework/robot.py`)
   - Hardware interface wrapper
   - Manages kick/walk channel exclusivity
   - Polls robot mode and fall state

### Key Parameters

Control parameters in `src/tactics/motion.py` (as of 2026-06-26):

```python
_ARRIVE_DISTANCE = 0.15      # m, distance threshold for "arrived"
_ARRIVE_ANGLE = 0.20         # rad (~11.5°), angle threshold for "arrived"
_TURN_THRESHOLD = 0.5        # rad (~28.6°), pure rotation vs forward+turn
_ANGULAR_SPEED_FLOOR = 0.25  # rad/s, minimum angular velocity
_ANGULAR_DEAD_ZONE = 0.15    # rad (~8.6°), no floor below this angle error
_LINEAR_SPEED_FLOOR = 0.3    # m/s, minimum linear velocity
```

Tuning parameters in `src/soccer_framework/config.py`:

```python
max_linear_speed: float = 0.8          # m/s
max_angular_speed: float = 1.0         # rad/s
yaw_avoid_bias_max: float = 0.6        # rad/s per neighbor
yaw_avoid_min_distance_m: float = 0.78 # m
```

## Common Pitfalls and Fixes

### 1. Don't set vy in combined commands
**Problem**: Setting `vy != 0` when `vx` or `vyaw` are also non-zero causes vy to be ignored, leading to unexpected motion.

**Solution**: Always use `MoveIntent(vx=..., vy=0.0, vyaw=...)` for combined motion.

### 2. Floor values causing overshoot
**Problem**: Large `_ANGULAR_SPEED_FLOOR` (e.g., 0.4 rad/s) amplifies small angle errors, causing oscillation at target.

**Symptoms**: Robot spins multiple circles at Ready position.

**Solution**: 
- Lower floor (now 0.25 rad/s)
- Add dead zone (now 0.15 rad) for pure proportional control at small errors
- See `note/motion_spinning_fix_applied.md` for details

### 3. Arrive threshold too strict
**Problem**: Small `_ARRIVE_ANGLE` (e.g., 0.12 rad = 6.9°) causes repeated micro-adjustments.

**Solution**: Relax threshold (now 0.20 rad = 11.5°) for smoother arrival.

### 4. Yaw avoidance stacking with floor
**Problem**: In Ready state with `avoid_opponents=True`, base vyaw (0.4) + avoidance bias (0.6) = 1.0 rad/s max, causing fast spinning.

**Solution**: Lowering floor to 0.25 reduces base contribution, leaving headroom for avoidance.

## Motion Control Workflow

When editing motion control code:

1. **Read kinematics constraints** (this skill, `note/motion_controller_design.md`)
2. **Understand the three-layer architecture**:
   - Path detour modifies target position
   - Walking control computes vx + vyaw to reach target
   - Yaw avoidance adds vyaw bias based on nearby robots
3. **Preserve `vy=0` in combined commands**
4. **Test parameter changes** against known scenarios:
   - Ready positioning (should not spin)
   - Ball chasing (should not get stuck)
   - Precise alignment for kicks
5. **Run motion tests**:
   ```bash
   python -m pytest tests/ -k "motion or ready or avoid" -xvs
   ```

## Files To Inspect

### Motion Control Core
- `src/tactics/motion.py`: `MotionController`, walking control, yaw avoidance
- `src/tactics/navigation.py`: `ObstacleCollector`, path detour via points
- `src/tactics/geometry.py`: coordinate transforms, field frame helpers
- `src/tactics/kick_hysteresis.py`: kick mode enter/exit hysteresis

### Robot Interface
- `src/soccer_framework/robot.py`: `RobotClient`, `TeamRobotManager`
- `src/soccer_framework/types.py`: `MoveIntent`, `KickIntent`, `RobotCommand`
- `src/soccer_framework/config.py`: `SoccerStrategyTuning` motion parameters

### Behavior Tree Nodes
- `src/behavior_tree/nodes/actions.py`: `GoReadyTarget`, `AvoidOpponentRestart`
- `src/play/nodes.py`: role action nodes (kick, hold position, etc.)

### Documentation & Analysis
- `note/motion_controller_design.md`: kinematics model explanation
- `note/motion_spinning_issue_analysis.md`: detailed spinning problem analysis
- `note/motion_spinning_fix_applied.md`: applied fix summary

## Testing

### Motion-Related Tests
```bash
# Ready positioning and avoidance
python -m pytest tests/test_soccer_framework_core.py::TeamFlowTests::test_ready_phase_walks_every_player_to_ready_target -xvs
python -m pytest tests/test_soccer_framework_core.py::AvoidanceTests -xvs

# All motion-related
python -m pytest tests/ -k "ready or avoid or motion" -x

# Full suite
python -m pytest tests/ -x
```

### Manual Verification Checklist
After motion control changes, verify:
- ✅ Ready state: robots stop at target without spinning
- ✅ Ready state with neighbors: avoidance active, still stable
- ✅ PLAY state: ball chasing not affected
- ✅ Precise alignment: kick approach targets still accurate
- ✅ Large angle turns: pure rotation still responsive

## Debugging Motion Issues

### Robot spinning at target
- Check `_ANGULAR_SPEED_FLOOR` and `_ANGULAR_DEAD_ZONE`
- Check `_ARRIVE_ANGLE` threshold
- Check if yaw avoidance is stacking with floor (Ready state)
- See `note/motion_spinning_issue_analysis.md`

### Robot not reaching target
- Check `_ARRIVE_DISTANCE` (too large?)
- Check if obstacle avoidance is blocking the path
- Check if robot is stuck in via-point loop

### Robot oscillating left-right
- Check yaw avoidance sensitivity (`yaw_avoid_min_distance_m`)
- Check if multiple neighbors are triggering conflicting biases

### Robot moving in wrong direction
- **First check**: are you setting `vy != 0` with `vx` or `vyaw`? (vy will be ignored)
- Check coordinate frame (field vs robot-relative)
- Check angle normalization

## Verification

For motion control code edits:

```bash
# Type check
python3 -m py_compile src/tactics/motion.py src/tactics/navigation.py src/soccer_framework/robot.py

# Motion tests
python3 -m pytest tests/test_soccer_framework_core.py::AvoidanceTests -xvs
python3 -m pytest tests/ -k "ready" -x

# Full suite
python3 -m pytest tests/ -x
```

For parameter tuning only (no code changes), run focused tests and consider simulation validation using `skills/soccersim-auto-debug/SKILL.md`.

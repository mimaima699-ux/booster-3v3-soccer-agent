# 重构方案：WorldState 数据流分层职责清理

> 状态：**已执行**（批次 1-3 全部完成）
> 日期：2026-06-23

## 一、问题诊断

### 问题 1：新鲜度过滤方式不统一（三种模式混用）

当前 DataLayer 对三类数据的过滤方式各不相同：

| 数据 | 过滤方式 | 过滤后位置 | 副作用 |
|---|---|---|---|
| `game_state` | 原地改 `world.game_state = None` + 写 `GAME` 黑板 | 两处都有 | `world.game_state` 被 None 污染后，`UpdateRules` 读到 None（恰好期望，但不显式） |
| `ball` | 不改 `world.ball`，只写 `BALL` 黑板（新值） | 只有 `BALL` 黑板 | **双源不一致**：`world.ball` 仍是原始过期值，`WORLD` 和 `BALL` 不同步 |
| `teammates`/`opponents` pose | 原地改 `world.teammates[i].pose = None` | 只有 `WORLD` | 无副作用，但和其他两者不统一 |

核心矛盾：`ball` 过滤后 `WORLD.ball`（过期值）和 `BALL`（None）不一致；而 `game_state`/`pose` 过滤后 `WORLD` 就是最终值。策略层有时读 `WORLD`，有时读 `BALL`，取到的语义不同。

### 问题 2：GAME/BALL 黑板 key 是 WORLD 的冗余派生

全代码搜索结果显示，`BlackboardKeys.GAME` / `BlackboardKeys.BALL` 被读取 **19 处**，分布在 `conditions.py`（13 处）、`actions.py`（6 处）、`nodes.py`（3 处，经 `_read_play_context`）。

它们本质上都是 `world.game_state` / `world.ball` 的副本，但过滤逻辑导致副本和原始值不一致（见问题 1）。

### 问题 3：penalty / ready_slot 数据重复

| 字段 | 真正读取处（不含日志） | 权威来源 | 存在重复 |
|---|---|---|---|
| `RobotState.penalty` | **0 处**（仅 `runtime.py:305` 日志用） | `GameControlState.teams[].players[].penalty` | 是 |
| `RobotState.ready_slot` | **0 处**（仅 `runtime.py:305` 日志用） | `config.ready_slot_for_player(player_id)` | 是 |

`RobotState` 上的 `penalty` 和 `ready_slot` 是纯冗余拷贝，策略逻辑从不读它们：
- "能否出场"走的是 `game.is_active_player()`（查 `GameControlState`）
- 槽位走的是 `config.ready_slot_for_player()`

### 问题 4：PlayContext 的 `game` 字段从未被读

全代码搜索确认：`ctx.game` 在所有 `.py` 文件中 **0 次读取**。只有 `ctx.world`（3 处）和 `ctx.ball`（1 处）被使用。

### 问题 5："能否活动"的判断跨数据源（领域固有，非缺陷）

| 判断 | 数据源 | 查询方式 | 是否重复 |
|---|---|---|---|
| 是否受罚 | `GameControlState`（裁判） | `game.is_active_player(team_id, player_id)` | 否 |
| 是否摔倒 | `RobotRuntimeStatus`（硬件） | `IsRobotFallen` 条件叶 | 否 |
| 是否 walk 模式 | `RobotRuntimeStatus`（硬件） | `IsNotWalkMode` 条件叶 | 否 |

这三者来自不同数据源（GameController vs 硬件），各自单一来源，无重复。**跨源是领域固有属性，不是设计缺陷**，无需消除。

---

## 二、设计原则

1. **单一数据源**：每条数据只有一个写入点，其他地方都是读引用。
2. **WorldState 是唯一的"已过滤快照"**：所有新鲜度过滤在 DataLayer 统一完成，原地修改 WorldState。策略层拿到的 `WORLD` 黑板值已经是过滤后的，不需要再判断新鲜度。
3. **不产生派生副本**：删除 `GAME`/`BALL` 黑板 key，它们只是 `world.game_state`/`world.ball` 的冗余拷贝。
4. **配置数据不进 WorldState**：`ready_slot` 是配置派生值，不属于"被观测的世界"，从 `RobotState` 删除。对手的 `ready_slot` 在 provider 中硬编码为 `SIDE`，并非真实信息，删除后不再误导。
5. **裁判数据只在 GameControlState**：`penalty` 从 `RobotState` 删除，权威源是 `GameControlState`。策略层已通过 `game.is_active_player()` 查询，删除不影响调用复杂度。
6. **硬件状态保持独立**：`RobotRuntimeStatus` 来自不同数据源（TeamRobotManager），不进 WorldState，保持独立黑板 key。

---

## 三、重构方案

### 第 1 步：RobotState 结构瘦身

**目标**：删除纯冗余字段 `penalty` 和 `ready_slot`。

#### 设计决策

**`penalty` 删除**：全代码搜索确认 `RobotState.penalty` 在策略逻辑中 **0 次读取**。所有判罚判断都走 `game.is_active_player(team_id, player_id)`（查 `GameControlState`），由 `kit.is_player_allowed` 包装后通过回调分发。`_sync_penalties_locked` 只是把 `GameControlState` 里的 penalty 拷贝到 `RobotState` 上，但没人读那个拷贝。删除不会增加任何调用复杂度。

**`ready_slot` 删除**：叶子节点实际全部通过 `config.ready_slot_for_player(player_id)` 读取槽位（~15 处），`RobotState.ready_slot` 在策略逻辑中 **0 次读取**（仅 `runtime.py:305` 日志用）。保留它会产生双源读取的不一致性。此外对手的 `ready_slot` 在 `world.py` 构造时硬编码为 `ReadySlot.SIDE`，并非真实信息——对手的槽位我们无法知道，留着反而误导。删除后：
- 叶子节点不受影响（已从 `config` 读）
- 日志改为从 `config.ready_slot_for_player()` 查
- 对手 `RobotState` 不再带无意义的 `ready_slot` 字段

#### 改前

```python
@dataclass
class RobotState:
    player_id: int
    ready_slot: ReadySlot       # ← 冗余：config 已有，且对手瞎填 SIDE
    pose: Pose2D | None = None
    penalty: Penalty = Penalty.NONE  # ← 冗余：GameControlState 已有
    last_seen_at: float = 0.0
```

#### 改后

```python
@dataclass
class RobotState:
    player_id: int
    pose: Pose2D | None = None
    last_seen_at: float = 0.0
```

#### 影响点

| 文件 | 改动 |
|---|---|
| `types.py` | 删除 `penalty`、`ready_slot` 字段 |
| `world.py` | 删除 `_sync_penalties_locked` 方法及 `set_game_state` 中的调用；构造 `RobotState`（队友和对手）时不传 `ready_slot=` |
| `runtime.py` | `_robot_record` 里 `robot.penalty.value` 改为从 `game_state` 查询；`robot.ready_slot.value` 改为从 `config.ready_slot_for_player()` 查 |
| `game_state.py` | 不变（操作 `PlayerState`，不是 `RobotState`） |
| 测试 | 构造 `RobotState` 时删掉 `ready_slot=` 和 `penalty=` 参数 |

#### 日志适配

```python
def _robot_record(robot, game, config, command, now):
    player = game.get_player_state(config.team_id, robot.player_id) if game else None
    record = {
        "player_id": robot.player_id,
        "ready_slot": config.ready_slot_for_player(robot.player_id).value,  # ← 从 config 查
        "penalty": player.penalty.value if player else "NONE",              # ← 从 game_state 查
        "pose": _pose_record(robot.pose),
        ...
    }
```

### 第 2 步：DataLayer 统一过滤，删除 GAME/BALL 黑板 key

**目标**：三类数据过滤方式统一为"原地改 WorldState"，不再产生 `GAME`/`BALL` 副本。

#### 改前 DataLayer

```
DataLayer (Sequence)
├── UpdateClock          ← 写 NOW
├── UpdateWorld          ← 写 WORLD + GAME（原始 game_state 副本）
├── UpdateGameState      ← 原地改 world.game_state + 写 GAME（过滤后副本）
├── UpdateRecentBall     ← 不改 world.ball，只写 BALL（过滤后副本）
├── UpdateRobotPoses     ← 原地改 world.teammates[i].pose = None
├── UpdateRules          ← 读 world.game_state
└── UpdateRobotStatus ×N ← 写 robot_status_key(N)
```

#### 改后 DataLayer

```
DataLayer (Sequence)
├── UpdateClock          ← 写 NOW                                  [不变]
├── UpdateWorld          ← 写 WORLD（原始）                        [不再写 GAME]
├── UpdateGameState      ← 原地改 world.game_state = None           [不再写 GAME]
├── UpdateRecentBall     ← 原地改 world.ball = None                 [不再写 BALL]
├── UpdateRobotPoses     ← 原地改 world.teammates[i].pose = None    [不变]
├── UpdateRules          ← 读 world.game_state                      [不变]
└── UpdateRobotStatus ×N ← 写 robot_status_key(N)                   [不变]
```

#### 黑板 key 变化

| 删除 | 保留 |
|---|---|
| `GAME`（→ `world.game_state`） | `NOW`、`WORLD`、`ROLES`、`SAFETY_ACTIVE`、`EXECUTOR`、`cmd_key(N)`、`robot_status_key(N)` |
| `BALL`（→ `world.ball`） | |

### 第 3 步：BT 叶子节点读取方式适配

**目标**：19 处 `blackboard.read(BlackboardKeys.GAME)` / `BALL` 改为从 `WORLD` 提取。

为避免重复样板代码，给共享基类加保护方法：

```python
class _ReadOnlyLeaf(py_trees.behaviour.Behaviour):
    def _read_world(self) -> WorldState | None:
        world = self.blackboard.read(BlackboardKeys.WORLD)
        return world if isinstance(world, WorldState) else None

    def _read_game(self) -> GameControlState | None:
        world = self._read_world()
        return world.game_state if world is not None else None

    def _read_ball(self) -> BallState | None:
        world = self._read_world()
        return world.ball if world is not None else None
```

`_ActionLeaf` 同样加这三个方法。

#### 适配示例

```python
# 改前
game = self.blackboard.read(BlackboardKeys.GAME)
ball = self.blackboard.read(BlackboardKeys.BALL)
if game is None or ball is None:
    return ...

# 改后
game = self._read_game()
ball = self._read_ball()
if game is None or ball is None:
    return ...
```

#### 影响文件

| 文件 | 改动量 |
|---|---|
| `conditions.py` | 13 处 `GAME` + 2 处 `BALL` 读取改为 `_read_game()`/`_read_ball()` |
| `actions.py` | 6 处 `GAME` + 3 处 `BALL` 读取改为 `_read_game()`/`_read_ball()` |
| `nodes.py` | `_read_play_context` 改为只读 `WORLD`（见第 4 步） |

### 第 4 步：PlayContext 简化

**目标**：`PlayContext` 只包一个 `world`，`game`/`ball` 变为派生属性。

#### 改前

```python
@dataclass(frozen=True)
class PlayContext:
    world: WorldState
    ball: BallState | None
    game: GameControlState

    @property
    def known_ball(self) -> BallState: ...
```

#### 改后

```python
@dataclass(frozen=True)
class PlayContext:
    world: WorldState

    @property
    def game(self) -> GameControlState | None:
        return self.world.game_state

    @property
    def ball(self) -> BallState | None:
        return self.world.ball

    @property
    def known_ball(self) -> BallState:
        if self.ball is None:
            raise ValueError("ball is required for this decision")
        return self.ball
```

#### `_read_play_context` 简化

```python
# 改前：读 3 个黑板 key + 类型检查
def _read_play_context(blackboard, require_ball=False) -> PlayContext | None:
    world = blackboard.read(BlackboardKeys.WORLD)
    ball = blackboard.read(BlackboardKeys.BALL)
    game = blackboard.read(BlackboardKeys.GAME)
    if not isinstance(world, WorldState) or not isinstance(game, GameControlState):
        return None
    if ball is not None and not isinstance(ball, BallState):
        return None
    if require_ball and ball is None:
        return None
    return PlayContext(world=world, ball=ball, game=game)

# 改后：读 1 个黑板 key
def _read_play_context(blackboard, require_ball=False) -> PlayContext | None:
    world = blackboard.read(BlackboardKeys.WORLD)
    if not isinstance(world, WorldState):
        return None
    if require_ball and world.ball is None:
        return None
    return PlayContext(world=world)
```

#### 影响文件

| 文件 | 改动 |
|---|---|
| `context.py` | `PlayContext` 定义改为只包 `world` + 派生属性 |
| `nodes.py` | `_read_play_context` 简化；已有 `ctx.world` / `ctx.ball` 调用不变 |
| `playbook.py` | 已有 `ctx.world` / `ctx.ball` 调用不变；`ctx.game` 如有间接使用走属性 |

---

## 四、重构后的数据流

```
ROS Topics
  ├── GameController topic → GameControlState (含 penalty, last_seen_at)
  ├── 队友 pose topic      → RobotState (pose, last_seen_at)
  ├── 对手 pose topic      → RobotState (pose, last_seen_at)
  └── ball pose topic      → BallState (x, y, last_seen_at)
          │
          ▼
  RosTruthProvider.get_snapshot() → WorldState (原始快照, deepcopy)
          │
          ▼
  ┌─ DataLayer ──────────────────────────────────────────────┐
  │ UpdateClock        → NOW                                  │
  │ UpdateWorld        → WORLD (原始)                         │
  │ UpdateGameState    → world.game_state = None (if stale)   │
  │ UpdateRecentBall   → world.ball = None (if stale)         │
  │ UpdateRobotPoses   → world.teammates[i].pose = None       │
  │ UpdateRules        → 读 world.game_state (已过滤)         │
  │ UpdateRobotStatus  → robot_status_key(N) (独立, 硬件源)   │
  └───────────────────────────────────────────────────────────┘
          │
          ▼
  WORLD = 唯一的已过滤快照 (game_state / ball / teammates / opponents 全部已过滤)
          │
          ├── conditions.py / actions.py → 读 world.game_state / world.ball
          ├── PlayContext(world=world)   → PLAY 策略层
          └── tree.py trace              → 日志
```

### 数据源归属表（重构后）

| 数据 | 权威来源 | 存储位置 | 过滤层 |
|---|---|---|---|
| 比赛状态 (state/phase/set_play/penalty) | GameController ROS topic | `WorldState.game_state` | `UpdateGameState` |
| 球位置 | ball pose topic | `WorldState.ball` | `UpdateRecentBall` |
| 队友位姿 | 队友 pose topic | `WorldState.teammates[i].pose` | `UpdateRobotPoses` |
| 对手位姿 | 对手 pose topic | `WorldState.opponents[i].pose` | `UpdateRobotPoses` |
| 队友/对手槽位 (ready_slot) | SoccerConfig | `config.ready_slot_for_player()` | 不过滤（配置值，仅队友有真实值） |
| 硬件状态 (mode/fall_down) | TeamRobotManager | `robot_status_key(N)` 黑板 | `UpdateRobotStatus` |
| 命令 (cmd) | BT 叶子写入 | `cmd_key(N)` 黑板 | 不过滤 |

**零重复**：每条数据只有一个写入点。

---

## 五、执行顺序

按依赖关系分 3 批，每批可独立测试：

| 批次 | 改动 | 影响范围 | 状态 |
|---|---|---|---|
| **批次 1** | 删 `RobotState.penalty` + `ready_slot`；删 `_sync_penalties_locked`；日志适配 | `types.py`、`world.py`、`runtime.py`、测试 | ✅ 完成 |
| **批次 2** | `UpdateRecentBall`/`UpdateGameState` 改为原地改 `world`；删 `GAME`/`BALL` 黑板 key；加 `_read_game()`/`_read_ball()` 辅助；19 处调用适配 | `data.py`、`blackboard.py`、`conditions.py`、`actions.py`、`nodes.py` | ✅ 完成 |
| **批次 3** | `PlayContext` 简化为只包 `world`；`_read_play_context` 简化 | `context.py`、`nodes.py`、`playbook.py` | ✅ 完成 |

---

## 六、不在本次范围内

- `WorldState.updated_at` 的语义重新定义（当前是"最后一次任意真值回调写入"，重构后仍保留，后续可讨论是否改为快照时间）
- `RosTruthProvider` 内部字段命名（`self._robots` → `self._teammates`，纯内部，不影响接口）
- `note/` 和 `docs/` 下文档中的 `world.robots` 引用更新（非代码，不影响运行）
- `_ReadOnlyLeaf` / `_ActionLeaf` 去掉下划线前缀（纯命名风格，影响面大，单独批次处理）
- `is_player_allowed` / `is_opponent_restart` 从 `SoccerKit` 挪到 `GameControlState`（架构偏好问题，需评估 `team_id` 传递方式）
- `tactics/targeting` 方法签名改为接收 `PlayContext` 而非拆散传 `world`/`ball`/`game`（影响所有 targeting 公共方法，单独批次处理）

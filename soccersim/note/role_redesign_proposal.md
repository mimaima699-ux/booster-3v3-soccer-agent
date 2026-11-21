# SoccerSim 角色 (Role) 设计重构方案

本方案对 SoccerSim 的角色策略接口进行重构，建立一个符合 **KISS** 原则的模块化
角色架构：**统一基类 + 共用工具节点 + 两层继承**。

---

## 1. 现有设计的痛点

重构前动态角色派生自两个割裂的基类：

* **`HoldTargetRole`（站位流）**：门将 / 后卫 / 支援手，只有一个 `target()` 接口。
* **`KickRole`（追踢流）**：追球手，内部是「检测踢球距离 → 踢 → 否则靠近」子树。

### 痛点一：基类割裂限制复合角色

很多角色职责是**复合**的——例如门将平时守门（站位流），球滚进禁区时出击解围
（追踢流）。基类割裂导致这类「状态/条件驱动的复合角色」无法通过继承单一基类
轻松实现。

### 痛点二：通过 Playbook 重写职责不直观

重构前门将出击靠 `Playbook.assign_roles` 把门将本帧分工强行改写成 `"chaser"`。
这在语义上不直观——**守门员出击解围时依然是守门员**，黑板角色标签频繁跳变增加
调试难度。

---

## 2. 方案：统一 `RoleStrategy` + 共用工具节点

### 2.1 唯一契约：`build_subtree`

`RoleStrategy` 是唯一基类，只承诺一件事——子类实现 `build_subtree` 返回本角色的
行为子树：

```python
class RoleStrategy:
    name: ClassVar[str] = ""

    def build_subtree(self, build: RoleBuildContext) -> Behaviour:
        """返回本角色的行为子树。子类必须实现。"""
        raise NotImplementedError
```

`target` / `wants_to_kick` / `kick_target` 等**不再是基类契约**，而是子类为配合
工具节点自行定义的实现方法。高阶角色若不复用工具节点，可完全不定义它们。

### 2.2 共用工具节点（默认零件）

`role.py` 提供三个共用叶节点和一个装配工厂，90% 的角色直接用它们拼装子树：

| 工具 | 作用 |
| --- | --- |
| `MoveToTarget(build, target_fn, ...)` | 每帧调 `target_fn` 算目标点，下发移动命令；自动 `clear_player` 清踢球迟滞 |
| `KickAction(build, kick_target_fn, ...)` | 每帧调 `kick_target_fn` 算瞄准点，下发踢球命令 |
| `WantsKick(build, wants_kick_fn)` | 条件叶：调 `wants_kick_fn` 判断本帧是否愿意踢球 |
| `build_attack_subtree(build, AttackSubtreeConfig(...))` | 工厂：组装「条件踢球 → 否则跑位」标准进攻子树 |

`build_attack_subtree` 产出的子树形态：

```text
Selector(Attack)
├── Sequence(KickBranch)
│   ├── WantsKick?          ← wants_kick_fn 非空时才挂
│   ├── InKickRange
│   └── KickAction
└── MoveToTarget
```

---

## 3. 三层用法（渐进式开发阶梯）

### 层级一：入门角色（纯站位）

90% 的角色只需定义 `target()` + 用 `MoveToTarget` 单叶：

```python
class SupporterRole(RoleStrategy):
    name = "supporter"

    def target(self, build, frame) -> Pose2D:
        return build.team.targeting.support_target(...)

    def build_subtree(self, build):
        return MoveToTarget(build, self.target, reason_fn=lambda _: "supporter hold")
```

### 层级二：复合角色（条件性踢球）

需要「平时站位、特定条件出击踢球」时，用 `build_attack_subtree`：

```python
class GoalkeeperRole(RoleStrategy):
    name = "goalkeeper"

    def target(self, build, frame) -> Pose2D:
        return build.team.ready_stance.goalkeeper_guard_target(frame.ball)

    def wants_to_kick(self, build, frame) -> bool:
        return frame.ball is not None and build.team.targeting.ball_in_own_defensive_area(frame.ball)

    def kick_target(self, build, frame) -> Pose2D:
        return Pose2D(build.team.field.opponent_goal_x(), 0.0, 0.0)

    def build_subtree(self, build):
        return build_attack_subtree(
            build,
            AttackSubtreeConfig(
                target_fn=self.target,
                kick_target_fn=self.kick_target,
                wants_kick_fn=self.wants_to_kick,
                require_ball=False,
            ),
        )
```

行为树看 `wants_to_kick` + `InKickRange` 决定本帧踢还是走守位点。**球员在黑板上的
角色名始终是 `"goalkeeper"`**，不再需要 `assign_roles` 临时改写。

### 层级三：高阶定制（完全自定义子树）

需要结构性不同的子树（多拍运球、自定义状态机、非 move/kick 命令）时，直接覆写
`build_subtree` 自行装配，工具节点仍可按需复用。此层不再受默认装配约束，但需自行
保证命令写入契约（移动前 `team.kicker.clear_player(player_id)`，命令统一写
`cmd_key(player_id)`）。

---

## 4. 预期收益

1. **降低认知负载**：开发者只需关心 `build_subtree` 一个契约；入门角色用
   `MoveToTarget`，复合角色用 `build_attack_subtree`，阶梯清晰。
2. **消灭角色跳变**：门将出击时黑板身份仍是 `"goalkeeper"`，日志追溯清晰。
3. **结构统一**：工具节点收拢了 `clear_player` / `cmd_key` / 球丢失兜底等样板逻辑，
   高阶角色复用工具节点即可继承这些保障。
4. **两层继承**：`RoleStrategy` 基类 + 具体角色直接继承，无中间层，符合 KISS。

# 3v3 规则

本文档主要给 coding agent 使用，用于修改、审查和调试本项目策略代码时快速对齐自动裁判规则。涉及比赛规则、裁判逻辑、合法站位、重开球、处罚原因或 GameController 语义时，优先读取项目 skill：

```text
skills/soccersim-rules/SKILL.md
```

当前规则以裁判机仓库中的实现文档为准：

```text
/home/ylb/booster_robotics/booster_studio/booster_studio_client/booster_robocup_sim/docs/automatic_game_control.md
```

如果本文档与裁判机实现不一致，应优先按裁判机实现处理。

## Coding Agent 快速入口

### 改策略前先确认

- 当前代码面对的是 `READY`、`SET`、`PLAYING`、`stopped=true`，还是某个 set play。
- 站位目标是否在场内、己方半场、并远离禁止区域。
- 当前 set play 是我方还是对方主罚，是否需要避让或避免直接射门。
- GameController/ROS/JSON 输入是否已经完成类型收窄，不要用 `Any` 掩盖规则字段不确定性。
- 如果改动会影响站位、追球、射门、重开球或处罚，补充或更新聚焦测试。

### 策略代码硬约束

|场景|策略应避免|
|---|---|
|`SET`|继续下发移动目标；位移超过 `0.08 m` 可能触发 `motionInSet`。|
|`PLAYING` 且 `stopped=true`|明显移动；位移超过 `0.3 m` 可能映射为 `motionInSet`。|
|对方 kick-off|进入中圈或越过半场，直到球权开放。|
|对方定位球|距离球小于 `1.45 m`；对方 goal-kick 时还要避开其防守罚球区。|
|我方 kick-off|按旧规则等待“不同队友触球”；当前实现只看 2 次稳定触球或 set play 超时。|
|我方 throw-in / indirectFreeKick|间接要求未满足时直接射门。|
|stuck ball|把卡球建模成机器人处罚；当前裁判机是约 `700 ms` 后清球。|

### 常用代码入口

- `src/soccer_framework/types.py`：GameController 枚举、场地常量、typed state、开发者可用命令/状态 dataclass 及 Pose2D 坐标几何纯函数。
- `src/soccer_framework/runtime.py`：运行时命令过滤、truth provider、robot manager 和 GameController bridge 使用。
- `src/soccer_framework/__init__.py`：策略开发者优先导入的数据和命令 API。
- `src/tactics/geometry.py`：场地方向、坐标变换、合法目标 clamp。
- `src/tactics/targeting/restart.py`：重开球避让目标。
- `src/strategy.py`：队伍策略入口。
- `src/trees/`：行为树数据、条件和动作节点。
- `tests/test_soccer_framework_core.py`、`tests/test_soccer_framework_runtime.py`：优先补充规则相关单元测试。

## 场地与比赛时间

仿真足球场使用 RoboCup Humanoid Soccer League M-Field 尺寸：

|项目|默认值|
|---|---:|
|场地长度|14.0 m|
|场地宽度|9.0 m|
|中圈半径|1.5 m|
|球门宽度|2.6 m|
|球门高度|1.8 m|
|球门深度|0.6 m|
|罚球区|底线向内 3.0 m，宽 6.0 m|
|球门区|底线向内 1.0 m，宽 4.0 m|
|比赛时长|600 s|

## 比赛状态

GameControl 主状态包括：

- `initial`：初始状态，只重置自动裁判运行态，不执行判罚。
- `ready`：准备站位阶段。
- `set`：就绪阶段，球已摆放，机器人必须静止。
- `playing`：比赛进行中。
- `finished`：比赛结束。

`playing` 状态下还可能出现 `stopped=true`。这表示裁判暂停当前争抢，例如出界后的摆球和重开准备。此时机器人不应继续明显移动。

## Auto Match

开启 Auto Match 后，裁判机会启用自动比赛流程：

- 自动摆球：kick-off、throw-in、goal-kick、corner-kick、stuck ball 清球。
- 自动罚下和罚下摆放。
- Ready 阶段自动进入 Set。
- Set 阶段 5 秒后自动进入 Playing。
- 自动判断进球、出界、定位球完成、罚时归场、stuck ball。

关闭 Auto Match 后，自动规则不执行，手动裁判按钮仍可操作 Ready、Set、Playing、Ball Free、Finish、Reset、set play、stop play 和 goal。

## 默认启用与禁用规则

Auto Match 会打开自动规则功能组，但仍会跳过默认禁用列表中的单条规则。

当前默认启用的主要规则：

|规则|说明|
|---|---|
|`score_goals`|自动判定进球，并处理 kick-off / free-kick 进球合法性。|
|`ball_out`|open play 出边线或底线后，自动设置 throw-in、goal-kick 或 corner-kick。|
|`free_kick_complete`|主罚方稳定触球后，自动结束当前 set play。|
|`penalize_illegal_position`|判罚 Set、kick-off 球权开放前、定位球避让相关非法站位。|
|`penalize_motion_in_set`|判罚 Set 阶段移动。|
|`penalize_incapable_robot`|判罚倒地过久或长时间无活动。|
|`enforce_penalty_reentry`|处理罚时未结束的非法归场。|
|`clear_stuck_ball`|清理卡住的球。|
|`penalize_stopped_motion`|判罚 `playing` 且 `stopped=true` 时的明显移动。|

当前默认禁用的单条规则：

|rule id|影响|
|---|---|
|`leaving_field`|默认不自动判罚机器人离场。|
|`excess_kickoff_players_outside_own_half`|默认不自动判罚 kick-off 主罚方中圈内越过半场的人数超过 1 个。|
|`excess_goal_area_defenders`|默认不自动判罚本方 goal area 内防守人数超过 3 个。|

## Ready 与 Set

### Ready 自动进入 Set

Ready 阶段会记录所有未罚下机器人的位置。之后如果每帧位移都不超过 `0.01 m`，并连续稳定 `10 s`，裁判机会提前进入 Set。

如果稳定条件未满足，kick-off Ready 的 `secondaryTime=45 s` 到期后也会进入 Set。

### Set 自动进入 Playing

进入 Set 后，Auto Match 会等待 `5 s`，然后自动进入 Playing。

Set 阶段机器人必须静止。进入 Set 时会记录机器人位置；之后位移超过 `0.08 m` 会被判 `motionInSet`。

## 触球记录

进球合法性、出界归属和定位球完成都依赖稳定触球记录。

- 同一机器人需要连续接触至少 2 帧。
- 接触持续时间至少 40 ms。
- 接触消失后保留 100 ms 去抖窗口。
- 被罚下机器人的触球会被忽略。
- `last_touch_side` 只在 `state=playing` 且 `stopped=false` 时更新。
- 没有稳定触球记录时，默认最后触球方为 `home`。

## 开球规则

### 开球方

以下情况通过 kick-off 重新开始比赛：

|场景|开球方|
|---|---|
|比赛开始|由比赛配置决定|
|进球后|被进球方|
|kick-off 违规重开|原开球方|

kick-off 时，球会被摆在中圈中心，位置为 `[0.0, 0.0, 0.2]`。

### 开球站位

Set 阶段和 kick-off 球权开放前，站位限制如下：

- 所有机器人必须在场地边界内，容差 `0.05 m`。
- 非主罚方必须留在己方半场，并且不能进入中圈。
- 主罚方原则上应在己方半场；kick-off 时，主罚方机器人可以位于中圈内并越过半场。
- 非 kick-off 的 Set 阶段，任何机器人进入中圈都会被判非法站位。

注意：裁判机当前默认禁用 `excess_kickoff_players_outside_own_half`，所以不会自动执行“主罚方最多 1 名机器人在中圈内越过半场”的单条人数上限规则。

### 开球后的球权开放

kick-off 进入 Playing 后，主罚方稳定触球会使球权开放。若 10 秒内没有完成有效触球，kick-off set play 到期，裁判机会清空 `setPlay` 和 `kickingSide`，双方都可以正常争抢。

球权开放前，kick-off 站位限制仍然有效。

### 开球后的进球限制

kick-off 方不能在间接要求未满足时直接得分。当前实现的合法条件是：

- 该 kick-off 已经超时结束；或
- 从该 kick-off 开始后累计稳定触球次数达到 2 次。

触球次数不要求来自本队，也不要求来自不同机器人。

如果 kick-off 方在条件未满足时进球，进球无效，裁判机会记录 `kickoff_offense_retake`，并由原 kick-off 方重新开球。

## 进球规则

当球心进入球门体积时判定为进球候选。默认判定范围：

- `abs(ball_y) < 1.3`
- `ball_z < 1.8`
- `ball_x` 在 `7.0..7.6` 或 `-7.6..-7.0`

如果进球合法，裁判机会：

- 得分方加 1 分。
- 写入 `goal` 事件。
- 下一次 kick-off 给被进球方。
- 进入重新开球流程。

如果进球前存在 kick-off 或 free-kick 进球限制，进球可能被判无效。

## 出界与重开

出界规则只在 `state=playing`、`stopped=false`、`setPlay=noSetPlay` 的 open play 中触发。

### 边线球

当球越过边线时，判定为 throw-in。默认阈值：

```text
abs(ball_y) > 4.635 m
```

throw-in 判给最后稳定触球方的对方。裁判机会把球放到出界位置附近的边线上，`z=0.11`。

throw-in 属于间接定位球。主罚方直接射入对方球门时，需要满足间接要求才算进球。

### 门球

当球越过底线但没有进球时，裁判机会先把球权判给最后稳定触球方的对方；如果获判方是该底线的防守方，则为 goal-kick。

默认底线出界阈值：

```text
abs(ball_x) > 7.135 m
```

goal-kick 摆球点为出界端球门区角点，默认：

```text
x = +/-6.0, y = +/-2.0, z = 0.11
```

goal-kick 不是间接定位球，直接射入对方球门可有效得分。

### 角球

当球越过底线但没有进球，且获判方不是该底线的防守方时，判定为 corner-kick。

corner-kick 摆球点为出界端场地角点，默认：

```text
x = +/-7.0, y = +/-4.5, z = 0.11
```

corner-kick 不是间接定位球，直接射入对方球门可有效得分。

### 重开流程和超时

throw-in、goal-kick、corner-kick 开始时，比赛保持 `state=playing`，但进入 `stopped=true` 的摆球/重开准备。摆球完成后裁判机恢复 set play，`secondaryTime=45 s`。

如果 set play 在 45 秒内没有完成，计时到 0 后裁判机会执行 `finish_set_play()`，清空 `setPlay` 和 `kickingSide`，球权开放。

## 定位球进球限制

### 间接定位球

当前间接定位球包括：

- `indirectFreeKick`
- `throwIn`

主罚方直接射入对方球门时，必须满足下列任一条件：

- 该 set play 已经超时结束；或
- 从该 set play 开始后累计稳定触球次数达到 2 次。

触球次数不区分本队/对方，也不区分是否同一名机器人。

如果条件未满足，进球无效，裁判机会记录 `goal_disallowed_indirect`，并判给防守方 goal-kick。

### 任意球直接射入自家球门

以下 set play 中，如果主罚方在 set play 未超时、累计稳定触球次数不超过 1 次时直接射入自家球门，不按乌龙球加分：

- `directFreeKick`
- `indirectFreeKick`
- `throwIn`
- `goalKick`
- `cornerKick`

裁判机会记录 `goal_disallowed_own_free_kick`，并判给对方 corner-kick。

### 定位球完成

当当前 set play 记录到主罚方移动/触球，并且本帧有主罚方稳定触球时，裁判机会执行 `ballFree`，清空当前 `setPlay` 和 `kickingSide`，并写入 `restart_complete`。

## 非主罚方避让

在 `directFreeKick`、`indirectFreeKick`、`throwIn`、`goalKick`、`cornerKick` 期间，非主罚方需要避让球附近区域。

默认避让判定：

- 非主罚方机器人到球距离小于 `1.45 m` 时，判非法站位。
- goal-kick 时，非主罚方机器人进入主罚方防守罚球区也会被判非法站位。

## Stopped Motion

当 `state=playing` 且 `stopped=true` 时，未罚下、非罚时归场宽限期内、且未倒地的机器人不能明显移动。

如果机器人相对 stopped 起始参考位置移动超过 `0.3 m`，会被判 `motionInSet`。当前实现会跳过 `upDot <= 0.45` 的倒地机器人，避免把倒地恢复动作误判为 stopped motion。

## Stuck Ball

当前裁判机的 stuck ball 处理是清球，不是处罚最近机器人。

触发条件：

- 存在稳定 ball contact。
- 同一接触机器人持续卡球。
- 球从候选起点移动不超过 `0.05 m`。
- 持续时间达到 `700 ms`。

触发后，裁判机会按机器人到球的方向，把球放到离机器人 `0.45 m` 的位置，`z=0.11`，并写入 `ball_reset` warning 事件。

## 违规与处罚

### 通用罚下机制

自动机器人规则会产生 penalty call，再由 GameControl 统一写入 player penalty。

|自动规则 call|写入 player penalty|默认罚时|是否增加本队 penaltyCounter|
|---|---|---:|---|
|`illegalPosition`|`illegalPositioning`|30 s|否|
|`motionInSet`|`motionInSet`|30 s|否|
|`incapableRobot`|`incapableRobot`|30 s|否|
|`leavingTheField`|`leavingTheField`|`30 + 15 * penaltyCounter` s|是|

罚时只在 `state=ready` 或 `state=playing` 且 `stopped=false` 时递减。罚时归零后，player penalty 变回 `noPenalty`。

刚结束罚时的机器人有 `10 s` 归场宽限期。宽限期内不会因为 Set 移动、非法站位等规则再次被罚下。

### 非法站位

以下情况会被判 `illegalPosition`：

- Set 阶段超出场地边界。
- Set 阶段不在本方半场，且不满足 kick-off 主罚方中圈例外。
- 非主罚方在 kick-off 时进入中圈或越过半场。
- kick-off 球权开放前违反开球站位。
- 定位球期间，非主罚方进入球附近避让区域。
- goal-kick 期间，非主罚方进入主罚方防守罚球区。

当前默认不自动判罚：

- 机器人离场。
- kick-off 主罚方中圈内越过半场的人数超过 1 个。
- 本方 goal area 防守人数超过 3 个。

### 无能力机器人

无能力机器人只在 `state=playing` 且 `stopped=false` 时检测。

倒地过久：

- `upDot <= 0.45` 后持续 `10 s`。
- 如果 `upDot >= 0.7`、回到非 fallen 区间、关节速度达到 `0.02`，或 `upDot` 相对恢复参考值上升至少 `0.05`，倒地计时会被重置。

长时间无活动：

- 机器人不处于 fallen 状态。
- 位移小于 `0.03 m`。
- yaw 变化小于 `0.05 rad`。
- 关节速度小于 `0.02`。
- 持续 `10 s` 后判 `incapableRobot`。

### 离场

`leaving_field` 当前默认禁用。如果手动启用，机器人在 `set` 或 `playing` 中超出场地边界外 `2.5 m` 会被判离场。默认阈值：

```text
abs(x) > 9.5 m 或 abs(y) > 7.0 m
```

离场罚时为 `30 + 15 * penaltyCounter` 秒，并会增加本队 `penaltyCounter`。

## 机器人回场

罚时结束后，机器人可以回到比赛。裁判机会优先使用记录中的合法位置；如果没有可用位置，则通过边线附近的回场点返回。

如果机器人在罚时未结束前进入场地范围内，会触发 `penalty_reentry`：

- 当前罚时被重置。
- 如果 `place_players=true`，机器人会再次被移出场地。
- 写入 `illegal_reentry_warning`。
- 非法归场 warning 会累计：2 次 warning 转 1 次 caution，2 次 caution 转 `sentOff`。
- `sentOff` 后 `secsTillUnpenalized=255`，本场无法正常回场。

默认场地内判定范围：

```text
abs(x) <= 7.05 m 且 abs(y) <= 4.55 m
```

## 比赛结果

比赛时间结束后，比分更高的一方获胜。双方比分相同则为平局。

## 当前未覆盖或非完整实现

自动裁判不是完整 RoboCup 规则书实现。当前仍未完整覆盖：

- pushing
- penalty kick
- drop ball
- timeout
- half switching
- Stop 期间倒地机器人起身/移动的严格处罚
- 机器人被推入 goal area 后的免责窗口
- 需要更细关节/姿态语义的规则

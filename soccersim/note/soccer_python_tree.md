Root (Parallel, policy=SuccessOnAll)
├── 1. field_dimensions_to_bb (FieldDimensionsToBlackboard)         # 每 tick 更新场地尺寸
├── 2. game_state_to_bb (GameControlStateToBlackboard)               # 每 tick 更新比赛状态
├── 3. self_location_to_bb (SelfLocationToBlackboard)                 # 每 tick 更新自身定位
├── 4. camera_info_to_bb (CameraInfoToBlackboard)                     # 每 tick 更新相机参数
├── 5. detection_to_bb (DetectionToBlackboard)                         # 每 tick 更新视觉检测
├── 6. ball_tracker_update (BallTrackerUpdate)                         # 每 tick 更新球估计状态
├── 7. obstacle_grid_to_bb (ObstacleGridToBlackboard)                 # 每 tick 更新障碍物栅格
├── 8. fall_down_state_to_bb (FallDownStateToBlackboard)               # 每 tick 更新倒地状态
├── 9. agent_game_control_state_to_bb (AgentGameControlStateToBlackboard) # 更新agent控制状态
├── 10. teammates_to_bb (TeammatesStateToBlackboard)                  # 每 tick 写入队友数据
├── 11. team_tactics_update (TeamTacticsUpdate)                       # 每 tick 汇总信念并动态分配战术角色
├── 12. visualBasedKickInfoPublisherDecorator (Decorator: FailureIsRunning)
│   └── visual_based_kick_info_publisher (VisualBasedKickInfoPublisher)
├── 13. visualization_publish (VisualizationPublishAction)            # 每 tick 发布可视化 Marker
├── 14. teammate_broadcast (TeammateStateBroadcast)                   # 每 tick 广播自身状态给队友
└── 15. OneShotEnterSoccerMode (Sequence, memory=True)
    ├── EnterSoccerModeOneShot (Decorator: OneShot)
    │   └── RetryEnterSoccerMode (Decorator: Retry 100 times)
    │       └── EnterSoccerMode (Action)                              # 进入足球动作模式
    └── GameGuard (Decorator: FailureIsRunning)                       # 比赛主逻辑守护
        └── GameParallel (Parallel, policy=SuccessOnAll)              # 比赛主子树
            ├── TriggerGuard (Decorator: FailureIsRunning)
            │   └── DetectRelocationTrigger (Action)                  # 重定位触发监测
            ├── RelocationGuard (Decorator: FailureIsRunning)
            │   └── RelocationExecutor (Sequence)                      # 全局重定位执行器
            │       ├── IsRelocationNeeded (Condition)
            │       └── CallEnterFieldAction (Action)                 # 调用重定位并清除标志
            └── GamingSelector (Selector)                             # 比赛状态主分发器
                │
                ├── IfSubstitute (Sequence)                           # 1. 替补状态
                │   ├── IsSubstitute (Condition)
                │   └── StandStill (Action)                           # 原地停止
                │
                ├── IfTimeout (Sequence)                              # 2. 暂停状态 (全局)
                │   ├── IsTimeout (Condition)
                │   └── PrepareForEnsureLocalized (Sequence)          # 停止并重新定位
                │       ├── StandStill (Action)
                │       ├── EnsureLocalized (Selector)
                │       │   ├── IsSelfLocationKnown (Condition)
                │       │   └── ScanForLocalization (Action)
                │       └── EnsureLookAtBallSubtree (Sequence)
                │           ├── EnsureLookAtBall (Selector)
                │           │   ├── IsBallPoseKnown (Condition)
                │           │   └── CamScanField (Action)
                │           └── TrackBallGuard (Decorator: FailureIsRunning)
                │               └── CamTrackBall (Action)
                │
                ├── IfGameStopped (Sequence)                          # 3. 停止状态 (全局)
                │   ├── IsGameStopped (Condition)
                │   └── StandStill (Action)                           # 原地停止
                │
                ├── InitialStateSequence (Sequence)                   # 4. INITIAL 状态
                │   ├── IsGameInState(GameState.INITIAL) (Condition)
                │   └── InitialStateSubtree (Sequence)
                │       └── PrepareForEnsureLocalized (Sequence)      # 停止、定位、看球 (同上)
                │
                ├── ReadyStateSequence (Sequence)                     # 5. READY 状态 (排队开球就位)
                │   ├── IsGameInState(GameState.READY) (Condition)
                │   └── ReadyStateSubtree (Parallel)
                │       ├── GoToReadyPosition (Action)                # 移动至 Ready 就位点
                │       └── EnsureLookAtBallSubtree (Sequence)        # 对准并追踪球 (同上)
                │
                ├── SetStateSequence (Sequence)                       # 6. SET 状态 (原地盯球)
                │   ├── IsGameInState(GameState.SET) (Condition)
                │   └── SetStateSubtree (Parallel)
                │       ├── StandStill (Action)                       # 原地静止
                │       ├── EnsureLookAtBallSubtree (Sequence)        # 对准并追踪球 (同上)
                │       └── UpdateBallInGameStateSet (Action)         # 更新摆球信息
                │
                ├── PlayStateSequence (Sequence)                      # 7. PLAYING 状态 (核心博弈阶段)
                │   ├── IsGameInState(GameState.PLAYING) (Condition)
                │   └── PlayStateSubtree (Selector)
                │       │
                │       ├── IfPenalized (Sequence)                    # 7.1 处罚状态
                │       │   ├── IsUnderPenalty (Condition)
                │       │   └── PrepareForEnsureLocalized (Sequence)  # 停止、定位、看球 (同上)
                │       │
                │       ├── IfTimeout (Sequence)                      # 7.2 暂停状态 (PLAY内)
                │       │   ├── IsTimeout (Condition)
                │       │   └── PrepareForEnsureLocalized (Sequence)  # 停止、定位、看球 (同上)
                │       │
                │       ├── CheckAndGetUp (Sequence, memory=True)      # 7.3 摔倒起身
                │       │   ├── IsFallDown (Condition)
                │       │   └── GettingUp (Action)                    # 倒地恢复动作 (最多重试2次)
                │       │
                │       ├── free kick sequence (Sequence)             # 7.4 拦截对方定位球/任意球
                │       │   ├── not fall down (Decorator: Inverter)
                │       │   │   └── IsFallDown (Condition)
                │       │   └── SetPlaySubtree (Sequence)
                │       │       ├── IsSetPlayActive (Condition)
                │       │       └── SetPlayPhase (Selector)
                │       │           ├── SetPlayStopped (Sequence)
                │       │           │   ├── IsGameStopped (Condition)
                │       │           │   └── StandStill (Action)
                │       │           └── OpponentSetPlayDefense (Sequence)
                │       │               ├── not stopped (Decorator: Inverter)
                │       │               │   └── IsGameStopped (Condition)
                │       │               ├── IsRestartForOpponent (Condition)
                │       │               └── DefendSetPlayWithBallTracking (Parallel)
                │       │                   ├── EnsureLookAtBallSubtree (Sequence) (同上)
                │       │                   └── ExecuteAssignedTacticalTarget (Action)  # 走向站墙或防守目标点
                │       │
                │       └── NormalPlaySequence (Sequence)             # 7.5 常规博弈 (分主攻与支援角色运行)
                │           ├── not fall down (Decorator: Inverter)
                │           │   └── IsFallDown (Condition)
                │           └── NormalPlay (Selector)
                │               │
                │               ├── PrimaryAttackerPlay (Sequence)    # 7.5.1 主攻手逻辑
                │               │   ├── IsPrimaryAttacker (Condition)
                │               │   ├── FindBall (Selector)           # 寻找球
                │               │   │   ├── IsBallPoseKnown (Condition)
                │               │   │   └── SearchAndScan (Parallel)
                │               │   │       ├── SearchBallByPattern (Action)  # 走特定巡逻路径
                │               │   │       └── EnsureLookAtBallSubtree (Sequence) (同上)
                │               │   └── AttackingWithBallTracking (Parallel)
                │               │       ├── EnsureLookAtBallSubtree (Sequence) (同上)
                │               │       └── waitForLegalMove (Sequence)
                │               │           ├── WaitingForLegallyMove (Action)
                │               │           └── ChooseAttackOrVisualKick (Selector)
                │               │               ├── VisualBasedKickSequence (Sequence)  # 近距离视觉精细对齐踢球
                │               │               │   ├── ShouldEnterVisualBasedKick (Condition)
                │               │               │   └── SwitchToVisualBasedKick (Action)
                │               │               └── AttackSequence (Sequence)          # 普通导航追球与射门
                │               │                   ├── GoToBallAttackingPosition (Action)
                │               │                   └── KickBall (Action)
                │               │
                │               └── SupportPlay (Sequence)            # 7.5.2 支援/防守队员逻辑
                │                   ├── not primary attacker (Decorator: Inverter)
                │                   │   └── IsNotPrimaryAttacker (Condition)
                │                   └── SupportWithBallTracking (Parallel)
                │                       ├── EnsureLookAtBallSubtree (Sequence) (同上)     # 身体移动的同时保持头部追踪球
                │                       └── ExecuteAssignedTacticalTarget (Action)        # 走向战术层分配的目标点
                │
                └── FinishStateSequence (Sequence)                     # 8. FINISHED 状态 (比赛结束)
                    ├── IsGameInState(GameState.FINISHED) (Condition)
                    └── FinishStateSubtree (Sequence)
                        └── StandStill (Action)                       # 零速原地站立
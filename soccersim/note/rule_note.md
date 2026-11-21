边线界外球 (Throw-in)   球超出左右边界       非罚球方要远离球 1.45 m

角球 (Corner Kick)  防守把球踢出自己球门线

球门球 (Goal Kick)  进攻把球踢出对方球门线

因出界或违规而重开定位球时 (界外球、门球、角球)
状态变化：比赛的 state 保持为 playing，但 stopped 被置为 True。
具体状态流转：
触发/摆球阶段：球出界时，比赛依然是 state=playing，但状态切为 stopped=True，并且 setPlay 被设为对应的重开类型（如 throwIn），开始 3 秒的平滑搬球动画。
定位球就绪阶段：搬球完毕后，裁判机保持 state=playing 且 stopped=True，设定 secondaryTime=45s，等待主罚方开球。
球权释放：主罚方碰球后，裁判机宣布 Ball Free，状态切回 stopped=False，setPlay 变回 noSetPlay。
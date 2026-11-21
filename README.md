# Booster 3v3 Soccer Agent

本仓库包含两条策略线（均基于 Booster 官方示例，用 Booster Studio 打开对应目录构建 `.agent`）：

| 路径 | 版本 | 定位 | 核心增强 |
|---|---|---|---|
| [`soccersim/`](soccersim/) | 1.2.0 | **行为树版 · 参赛主线** | py_trees 行为树；分档踢球力度、射门瞄死角、传球评分/带球、落后压上 |
| 根目录 | 1.3.0 | 简单框架版 · 备用 | 两次触球开球、守门主动拦截、踢球决策梯（射门/传球/带球）、防无活动罚下 |

---

Simplified Version of Booster Champion 3v3 Soccer Tournament agent

This strategy framework has been greatly simplified to make it easier for players to implement, but it still needs optimization to achieve the best results. Players should try it out thoroughly in a simulation environment and only use it in the competition after successful debugging.

# How to use
1. Download [Booster Studio](https://studio.booster.tech/)
2. Open this project from Booster Studio
3. Waiting for environment prepare
4. Select a Virtual Robot
4. Just click "Activate, build, deploy and run agent"

# References
[BoosterOS Developer API Document](https://booster.feishu.cn/wiki/Fjj9wJS4OivUSaksW2HctpVonTh)
[Booster Agent Developer Guide](https://itq6ucn0mk.feishu.cn/wiki/J0FywI2CdinfU0kKLKycNiqfnwc?from=from_copylink)
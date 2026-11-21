# 项目 Agent 规则

这个仓库会作为模板/示例代码使用，所以代码不仅要能运行，也要在编辑器里保持干净，不留下明显的类型飘红。

- Python 代码要尽量保持 Pylance/pyright 友好。不要留下常见 IDE 会提示的类型错误。
- 处理外部输入或宽泛类型时，先做明确的类型收窄，再传给要求具体类型的 API。
- 不要用 `Any` 掩盖缺失依赖或类型问题；模板依赖的包应直接导入。
- 优先保持代码简单，但不要为了省几行代码用宽泛的 `cast` 掩盖类型问题，除非边界处已经完成了明确校验。
- 涉及比赛规则、裁判逻辑、策略约束、合法站位、重开球、处罚原因或 GameController 语义时，先读取并遵循项目本地 skill：`skills/soccersim-rules/SKILL.md`；其中 `note/3v3_rule.md` 是当前正式比赛规则摘要，若它与其他规则文档不一致，优先遵循 `note/3v3_rule.md`。
- 涉及运动控制、速度计算、避障逻辑、机器人运动学约束、行走参数调整时，先读取并遵循项目本地 skill：`skills/soccersim-motion/SKILL.md`；特别注意双足机器人的速度控制约束（组合使用 vx+vy+vyaw 时 vy 会被忽略，应只用 vx+vyaw）。
- 修改后在可行时运行相关语法检查和聚焦测试。
- 当用户提到自动调试、自动部署、自动跑比赛、策略迭代、评估踢球质量、对比 `artifacts/auto_debug` 报告，或要求使用固定仿真容器 `c972daf7bf19` 时，先读取并遵循项目本地 skill：`skills/soccersim-auto-debug/SKILL.md`。

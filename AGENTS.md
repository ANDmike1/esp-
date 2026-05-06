# NextBoard 硬件方案 Skill（本仓库引用）

本仓库已集成 [NextBoard](https://github.com/LeoKemp223/NextBoard) 的 **hardware-solution** 技能，供 Cursor Agent 做器件选型、BOM 风险、原理图级方案与评审流程时使用。

## 核心规则

- 在对话中可输入 **`$hardware-solution`** 作为触发语，或说明「按 hardware-solution 流程做硬件方案」。
- 器件参数须以数据手册或分销商页面为准，禁止凭记忆杜撰。
- 关键选择须说明取舍；风险清单与验证门控见技能内文档。

## 本仓库路径

| 内容 | 路径 |
|------|------|
| 技能入口 | `.cursor/skills/hardware-solution/SKILL.md` |
| 工作流与模板 | `.cursor/skills/hardware-solution/references/` |
| 独立评审（可选） | `.cursor/agents/hardware-reviewer.md` |

方案输出中若需落地到固定目录，可按 `SKILL.md` 内说明使用（如 `docs/hardware/datasheets/` 等）；本仓库无该目录时由 Agent 按需创建。

## 与 `D:\PCB画板\NextBoard` 的关系

项目内为**副本**；更新上游请重新从 `D:\PCB画板\NextBoard\skills\hardware-solution` 覆盖复制，或重新 `git pull` 后同步到 `.cursor/skills/hardware-solution`。

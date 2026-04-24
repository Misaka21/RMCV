---
name: rmcv-git-commit
description: Generate or execute RMCV2026 Chinese Git commit messages using emoji + conventional type style. Use when the user asks to commit, generate a commit message, summarize staged changes, or use `✨ feat`, `🐛 fix`, `🔧 chore`, and similar formats in /media/nuc11/common/RM/RMCV/RMCV2026.
---

# RMCV Git Commit

Use this format:

```text
<emoji> <type>[optional scope]: <description>

[optional body]

[optional footer(s)]
```

## Type Mapping

- `✨ feat`: 新增功能或能力
- `🐛 fix`: 修复 bug、错误行为或回归
- `📝 docs`: 文档、说明、AI 指令
- `💄 style`: 纯格式调整，不改变行为
- `♻️ refactor`: 重构，不新增功能，不修 bug
- `⚡️ perf`: 性能、延迟、吞吐、资源占用优化
- `✅ test`: 测试或验证工具
- `🔧 chore`: 仓库维护、清理、杂项
- `🔨 build`: CMake、构建、依赖装配
- `👷 ci`: CI/CD 工作流
- `💥 <type>!`: 破坏性变更，必须写 `BREAKING CHANGE:`

## Scope Suggestions

Prefer one clear scope:

- `repo`
- `docs`
- `config`
- `aimer`
- `auto_aim`
- `detector`
- `predictor`
- `fire_control`
- `auto_buff`
- `common`
- `transformer`
- `hardware`
- `serial`
- `camera`
- `plugin`
- `param`
- `umt`
- `test`
- `scripts`
- `simulator`

Omit scope if the change spans the whole repository and a scope would be misleading.

## Workflow

1. Run `git status --short` to inspect changed files.
2. If committing, inspect `git diff --staged`; if nothing is staged, inspect `git diff`.
3. Do not include unrelated local changes in the commit.
4. Pick the dominant intent and one type.
5. Write the subject in Chinese, concise and specific.
6. Add a body only when it explains important intent, risk, verification, or migration.
7. If user asked to commit, run `git add` only for relevant files and then `git commit`.

## Rules

- Description must be Chinese.
- Put emoji first.
- Keep subject usually under 50 Chinese characters.
- Do not invent issue IDs, reviewers, test results, or impact.
- Do not add `Generated with ...`.
- Do not add `Co-Authored-By` for AI tools.
- Do not add any AI attribution line.

## Examples

```text
📝 docs(repo): 拆分 AI 协作说明
```

```text
🐛 fix(serial): 修复瞄准模式字节转换边界
```

```text
♻️ refactor(predictor): 拆分战场快照更新流程
```

```text
🔨 build: 保持 TensorRT 检测器可选构建
```

```text
💥 refactor(param)!: 调整运行时参数路径

BREAKING CHANGE: 旧的 AutoAim.Predictor 参数路径不再读取，需要同步更新 config/aimer.toml。
```

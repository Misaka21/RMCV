---
name: rmcv-collab-workflow
description: Enforce the RMCV2026 repository collaboration workflow for /media/nuc11/common/RM/RMCV/RMCV2026, including module-based task splitting, branch naming, commit message style, PR requirements, build/test gates, and merge policies. Use whenever working on code, docs, branches, commits, reviews, or pull requests in this repository.
---

# RMCV2026 Collaboration Workflow

Follow this workflow when operating in `/media/nuc11/common/RM/RMCV/RMCV2026`.

## 1. Identify the Module Boundary

Pick the primary scope before editing:

- `aimer`: algorithm layer.
- `auto_aim`: armor auto-aim business chain.
- `detector`: armor or buff detection.
- `predictor`: tracking, EKF, target prediction.
- `fire_control`: target selection, ballistic solving, fire decision.
- `auto_buff`: energy mechanism.
- `common`: shared robot state, transformer, trajectory, latency, filters.
- `hardware`: camera, serial, sync frame.
- `serial`: protocol bytes and transceiver.
- `plugin`: logging, params, stats, visualizer, bag, watchdog.
- `param`: runtime/static parameter system.
- `umt`: message and shared object infrastructure.
- `config`: shared TOML/YAML config.
- `test`: validation tools.
- `scripts`: systemd, watchdog, cleanup, model scripts.

Keep changes inside one scope unless the task requires a cross-module contract change.

## 2. Sequence Cross-Module Work

Use this order for broad changes:

1. shared type or contract in `aimer/common`, `hardware`, `plugin`, or `umt`
2. producer module
3. consumer module
4. config and tests
5. docs and AI instructions

For serial/business mode changes, preserve the boundary:

```text
hardware/serial uint8_t -> hardware::SyncFrame -> aimer::RobotState -> business modules
```

Do not put `aimer::AimMode` or business strategy in `hardware/serial`.

## 3. Branch Rules

Use:

```text
<type>/<scope>-<short-desc>
```

Allowed `type`:

- `feat`
- `fix`
- `refactor`
- `docs`
- `test`
- `chore`
- `build`
- `ci`
- `perf`

Examples:

```text
refactor/predictor-battlefield-snapshot
fix/serial-aim-mode-boundary
docs/ai-instruction-split
```

Repository policy:

- `master`: stable deployable branch.
- `dev`: integration branch for large changes.
- feature branches are based on `dev` during major rewrites.
- do not force-push `master` or `dev`.

## 4. Commit Rules

Use `$rmcv-git-commit` for commit message generation and actual commits.

Never include AI attribution lines such as `Generated with ...` or `Co-Authored-By`.

## 5. PR Requirements

A PR should include:

1. scope and touched modules
2. behavior change summary
3. risk points
4. verification commands and results
5. rollback plan for hardware/runtime/config changes

For `config/*.toml`, `CMakeLists.txt`, `main.cpp`, `plugin/param`, `umt`, or protocol changes,
call out compatibility impact explicitly.

## 6. Build and Test Gates

Before merge, at minimum run:

```bash
cmake --build build -j$(nproc)
```

Run targeted tests based on scope:

```bash
./build/test_param
./build/test_transformer
./build/test_serial
./build/test_camera
./build/test_hardware
./build/test_fire_control
./build/test_ballistic
./build/test_playback
```

If dependencies are missing, report the exact blocker and which tests were not run.

## 7. Config Guardrails

- Runtime parameters must be read at the use point with `runtime_param::get_param<T>()`.
- Do not cache runtime params in constructors, `init()`, static locals, helper wrappers, or config structs.
- Preserve TOML types: `2.0` for double, `2` for int64, `false` for bool.
- Avoid committing personal machine paths or one-off tuning values.

## 8. Source of Truth

When uncertain, read the nearest `CLAUDE.md` in the working directory and the root
`CLAUDE.md`.

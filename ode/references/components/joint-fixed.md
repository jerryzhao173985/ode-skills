# Joint: Fixed  (`dJointCreateFixed` + `dJointSetFixed`)

> Source of truth: include/ode/objects.h:1766. Cited by file:line; do not invent symbols.

## What it is

A fixed joint locks two bodies (or a body to the static environment) so they maintain their **current relative pose** — **0 DOF**. After attaching, you call `dJointSetFixed` once to capture and lock the bodies' present relative transform (`include/ode/objects.h:2451`). It is mainly a debugging/assembly aid rather than a physically motivated constraint.

## Create & attach

```c
dJointID joint = dJointCreateFixed (world, 0);
dJointAttach (joint, body1, body2);                /* REQUIRED; 0 for a body => static env */
dJointSetFixed (joint);                            /* lock CURRENT relative pose; call after attach */
```

Verbatim shape from `ode/demo/demo_joints.cpp:228-230`:

```c
joint = dJointCreateFixed (world,0);
dJointAttach (joint,body[0],body[1]);
dJointSetFixed (joint);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetFixed` | `void dJointSetFixed (dJointID)` | include/ode/objects.h:2451 | Lock the joint to the bodies' current relative transform; must be called after attach. |
| `dJointGetFixedParam` | `dReal dJointGetFixedParam (dJointID, int parameter)` | include/ode/objects.h:3144 | Read back a fixed-joint `dParam*` value. |

## Parameters / conventions

- **No anchor, no axis**: a fixed joint has neither — `dJointSetFixed` snapshots the full relative pose at call time, so the **order matters**: attach first, then `dJointSetFixed`.
- The only `dParam*` exposure is via the matching `dJointSetFixedParam` / `dJointGetFixedParam` pair (read side cited at `include/ode/objects.h:3144`); the relevant constants are the CFM/ERP tuning members of the `dParam*` group (`include/ode/common.h:449-450`). There are no stops or motor on a fixed joint.

## Pitfalls

- Skipping `dJointSetFixed` after attach means the joint does **not** lock the current pose — it must be called (after attaching) to capture the relative transform (`include/ode/objects.h:2451`).
- Calling `dJointSetFixed` **before** `dJointAttach` captures the wrong (or no) relative pose; attach first (`include/ode/objects.h:1866`, `include/ode/objects.h:2451`).
- Forgetting `dJointAttach` altogether leaves the joint in limbo with no effect (`include/ode/objects.h:1688`).

## Never invent

- `dJointSetFixedAnchor` / `dJointSetFixedAxis` — a fixed joint has no anchor or axis; pose is captured by `dJointSetFixed`.
- `dJointFix` / `dJointLock` — the function is `dJointSetFixed`.

See also the joints overview: `references/joints.md`.

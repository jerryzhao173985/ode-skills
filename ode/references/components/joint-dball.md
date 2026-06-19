# Joint: Double Ball (`dJointCreateDBall`)

> Source of truth: include/ode/objects.h:1800. Cited by file:line; do not invent symbols.

## What it is

A double-ball ("distance") joint holds two anchor points (one on each body) at a fixed target distance, like a rigid rod between two ball-and-socket ends — it removes 1 DOF (the separation along the line between the anchors) while leaving the bodies free to swing. Cited by include/ode/objects.h:1800, include/ode/objects.h:3391, include/ode/objects.h:3397.

## Create & attach

```c
dJointID j = dJointCreateDBall(world, 0);     /* group 0 = normal alloc */
dJointAttach(j, body1, body2);                /* required; 0 for a body = static env */
dJointSetDBallAnchor1(j, x1, y1, z1);         /* anchor on body1, world coords */
dJointSetDBallAnchor2(j, x2, y2, z2);         /* anchor on body2, world coords */
dJointSetDBallDistance(j, 1.5);               /* target separation */
```

Cited by include/ode/objects.h:1800, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:3367 (anchor1), include/ode/objects.h:3373 (anchor2), include/ode/objects.h:3397 (distance).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetDBallAnchor1` | `void dJointSetDBallAnchor1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3367 | Set anchor1 (on body1) for the double-ball joint. |
| `dJointSetDBallAnchor2` | `void dJointSetDBallAnchor2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3373 | Set anchor2 (on body2) for the double-ball joint. |
| `dJointGetDBallAnchor1` | `void dJointGetDBallAnchor1 (dJointID, dVector3 result)` | include/ode/objects.h:3379 | Get anchor1 into `result`. |
| `dJointGetDBallAnchor2` | `void dJointGetDBallAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:3385 | Get anchor2 into `result`. |
| `dJointGetDBallDistance` | `dReal dJointGetDBallDistance (dJointID)` | include/ode/objects.h:3391 | Get the target distance. |
| `dJointSetDBallDistance` | `void dJointSetDBallDistance (dJointID, dReal dist)` | include/ode/objects.h:3397 | Set the target distance. |
| `dJointSetDBallParam` | `void dJointSetDBallParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:3403 | Set a double-ball `dParam*` parameter. |
| `dJointGetDBallParam` | `dReal dJointGetDBallParam (dJointID, int parameter)` | include/ode/objects.h:3409 | Read back a double-ball `dParam*` value. |

## Parameters / conventions

- Anchors are set in world (global) coordinates, one per body, via `dJointSetDBallAnchor1`/`Anchor2`. Cited by include/ode/objects.h:3364, include/ode/objects.h:3370.
- The constrained quantity is the distance between the two anchors; set/read it with `dJointSetDBallDistance`/`dJointGetDBallDistance`. Cited by include/ode/objects.h:3388, include/ode/objects.h:3394.
- Constraint tuning (e.g. soft constraint behavior) goes through `dJointSetDBallParam` using `dParam*` constants. Cited by include/ode/objects.h:3403.

## Pitfalls

- Set both anchors *after* attaching; an unattached joint is in limbo and has no effect. Cited by include/ode/objects.h:1688.
- The joint targets a *distance*, not a rigid lock — to fix the separation, set `dJointSetDBallDistance` explicitly; it is otherwise derived from the initial anchor positions. [VERIFY] (initial-distance defaulting is not stated in the header doc block at include/ode/objects.h:3388-3397).
- There is no axis to set on a double-ball joint — only anchors and distance. Cited by include/ode/objects.h:3364.

## Never invent

- `dJointSetDBallAxis` — a double-ball joint has no axis (that belongs to the double-hinge). Cited by include/ode/objects.h:3415 (DHinge axis, not DBall).
- `dJointCreateDistance` — the function is `dJointCreateDBall`. Cited by include/ode/objects.h:1800.
- `dJointSetDBallAnchor` (un-numbered) — only `Anchor1`/`Anchor2` exist. Cited by include/ode/objects.h:3367.

Cross-link: references/joints.md (joints overview).

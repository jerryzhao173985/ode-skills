# Joint: Hinge-2  (`dJointCreateHinge2`)

> Source of truth: include/ode/objects.h:1725. Cited by file:line; do not invent symbols.

Cross-link: see the joints overview at `references/joints.md` for the dParam family, joint lifecycle, and contact joints.

## What it is

The hinge-2 is a 2-DOF joint: one rotation about **axis 1** plus one rotation about **axis 2**, with a suspension spring between them. It is the classic car-wheel joint — axis 1 is the **steering** axis and axis 2 is the wheel-**rolling** axis. It requires two bodies to work (include/ode/objects.h:38 summary; "requires two bodies to work" — include/ode/objects.h:38 in inventory, function at include/ode/objects.h:1725).

## Create & attach

```c
// "front and back wheel hinges" — one hinge2 per wheel, chassis=body[0]
dReal zunit[3] = {0,0,1};   // axis1 = steering (vertical)
dReal yunit[3] = {0,1,0};   // axis2 = wheel rolling
dJointID j = dJointCreateHinge2 (world, 0);
dJointAttach (j, chassis, wheel);                  // body1=chassis, body2=wheel
const dReal *a = dBodyGetPosition (wheel);
dJointSetHinge2Anchor (j, a[0], a[1], a[2]);       // anchor at the wheel center
dJointSetHinge2Axes  (j, zunit, yunit);            // axis1 then axis2
```
Cited: `dJointCreateHinge2`/`dJointAttach`/`dJointSetHinge2Anchor`/`dJointSetHinge2Axes` per ode/demo/demo_buggy.cpp:257-261.

## Suspension + stops pattern (the most-used, most-misused)

```c
// soft suspension via the hinge2-only suspension params
dJointSetHinge2Param (j, dParamSuspensionERP, 0.4);
dJointSetHinge2Param (j, dParamSuspensionCFM, 0.8);

// lock a non-steered (rear) wheel along the steering axis with zero stops
dJointSetHinge2Param (j, dParamLoStop, 0);   // axis1 low  stop
dJointSetHinge2Param (j, dParamHiStop, 0);   // axis1 high stop
```
Cited: suspension at ode/demo/demo_buggy.cpp:266-267; lock-via-stops at ode/demo/demo_buggy.cpp:273-274. The demo explicitly notes that locking via `dParamVel`+`dParamFMax=dInfinity` "is no good as the wheels may get out of alignment" (ode/demo/demo_buggy.cpp:275-278) — prefer zero Lo/Hi stops.

```c
// per-step driving (axis2) + steering servo (axis1)
dJointSetHinge2Param (j, dParamVel2,  -speed);   // axis2: roll the wheel
dJointSetHinge2Param (j, dParamFMax2,  0.1);     // axis2 motor must have FMax2>0
dReal v = steer - dJointGetHinge2Angle1 (j);     // P-servo on axis1 angle
dJointSetHinge2Param (j, dParamVel,  v);         // axis1: steer
dJointSetHinge2Param (j, dParamFMax, 0.2);
dJointSetHinge2Param (j, dParamLoStop, -0.75);   // axis1 steering limit
dJointSetHinge2Param (j, dParamHiStop,  0.75);
dJointSetHinge2Param (j, dParamFudgeFactor, 0.1);
```
Cited: ode/demo/demo_buggy.cpp:171-183.

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetHinge2Anchor` | `void dJointSetHinge2Anchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2070 | Set the anchor point in world coordinates. |
| `dJointSetHinge2Axes` | `void dJointSetHinge2Axes (dJointID j, const dReal *axis1, const dReal *axis2)` | include/ode/objects.h:2084 | Set both axes at once (axis1=steering, axis2=rolling); pass NULL to keep one. |
| `dJointSetHinge2Param` | `void dJointSetHinge2Param (dJointID, int parameter, dReal value)` | include/ode/objects.h:2110 | Set a limit/motor/suspension param (axis2 via the suffixed forms). |
| `dJointAddHinge2Torques` | `void dJointAddHinge2Torques(dJointID joint, dReal torque1, dReal torque2)` | include/ode/objects.h:2118 | Apply torque1 about axis 1 and torque2 about axis 2. |
| `dJointGetHinge2Anchor` | `void dJointGetHinge2Anchor (dJointID, dVector3 result)` | include/ode/objects.h:2663 | Get the anchor as seen on body 1. |
| `dJointGetHinge2Anchor2` | `void dJointGetHinge2Anchor2 (dJointID, dVector3 result)` | include/ode/objects.h:2673 | Get the anchor as seen on body 2 (to measure joint separation). |
| `dJointGetHinge2Axis1` | `void dJointGetHinge2Axis1 (dJointID, dVector3 result)` | include/ode/objects.h:2679 | Get axis 1. |
| `dJointGetHinge2Axis2` | `void dJointGetHinge2Axis2 (dJointID, dVector3 result)` | include/ode/objects.h:2685 | Get axis 2. |
| `dJointGetHinge2Param` | `dReal dJointGetHinge2Param (dJointID, int parameter)` | include/ode/objects.h:2691 | Read back a param value. |
| `dJointGetHinge2Angle1` | `dReal dJointGetHinge2Angle1 (dJointID)` | include/ode/objects.h:2697 | Get the axis-1 angle (used for the steering servo). |
| `dJointGetHinge2Angle2` | `dReal dJointGetHinge2Angle2 (dJointID)` | include/ode/objects.h:2703 | Get the axis-2 angle. |
| `dJointGetHinge2Angle1Rate` | `dReal dJointGetHinge2Angle1Rate (dJointID)` | include/ode/objects.h:2709 | Get the axis-1 angular rate. |
| `dJointGetHinge2Angle2Rate` | `dReal dJointGetHinge2Angle2Rate (dJointID)` | include/ode/objects.h:2715 | Get the axis-2 angular rate (wheel spin speed). |

Deprecated (do not use in new code): `dJointSetHinge2Axis1` (include/ode/objects.h:2094) and `dJointSetHinge2Axis2` (include/ode/objects.h:2104) — both marked `ODE_API_DEPRECATED`, replaced by `dJointSetHinge2Axes`.

## Parameters / conventions

- **dParam family** (include/ode/common.h:441-454): `dParamLoStop`/`dParamHiStop` (axis-1 stops), `dParamVel`/`dParamFMax`/`dParamFudgeFactor`/`dParamBounce`/`dParamCFM`/`dParamStopERP`/`dParamStopCFM`.
- **Suspension** is hinge-2 only: `dParamSuspensionERP` (include/ode/common.h:453) and `dParamSuspensionCFM` (include/ode/common.h:454). The inventory notes these are "currently only implemented on the hinge-2 joint."
- **Axis convention:** axis 1 = the first vector to `dJointSetHinge2Axes` (steering), axis 2 = the second (rolling).
- **Second-axis params use the `2`-suffixed names.** The header documents `(dParamGroup2 | dParamFMax) == dParamFMax2` (include/ode/common.h:460-463), and the buggy demo drives the wheel with `dParamVel2`/`dParamFMax2` (ode/demo/demo_buggy.cpp:171-172).

## Pitfalls

- A motor on axis 2 does nothing unless `dParamFMax2 > 0`; setting only `dParamVel2` produces no force (inventory mistake; base form at include/ode/common.h:446).
- Passing the base `dParamVel` to `dJointSetHinge2Param` addresses **axis 1 only** — use `dParamVel2` for the wheel-spin axis (inventory mistake; ode/demo/demo_buggy.cpp:171 vs 179).
- To lock a rear wheel, set `dParamLoStop=dParamHiStop=0`; the alternative `dParamVel=0, dParamFMax=dInfinity` lets wheels drift out of alignment (ode/demo/demo_buggy.cpp:275-278).
- Forgetting `dJointAttach` leaves the joint in limbo with zero effect; hinge-2 specifically needs two real bodies (include/ode/objects.h:1725; attach rule include/ode/objects.h:1866).
- **Anchor vs Anchor2 legitimately diverge under soft suspension — do not treat it as a blown joint.**
  Because the suspension is a spring along axis 1, `dJointGetHinge2Anchor` (body 1, include/ode/objects.h:2663)
  and `dJointGetHinge2Anchor2` (body 2, include/ode/objects.h:2673) separate by the live suspension travel —
  several cm at vehicle scale is healthy. If you use that separation as a joint-integrity check, threshold
  at **vehicle scale (~0.1–0.2 m), not sub-mm**; a truly failed joint diverges to metres. (Empirically
  established in the buggy demo; a sub-mm threshold false-positives on a working soft suspension.)
- **axis1 and axis2 must be linearly independent — parallel axes ABORT ODE.** The hinge-2 builds its frame
  by perpendicular-izing the second axis against the first; parallel (steering ∥ rolling) axes collapse it to
  zero, ODE's safe-normalize fails, and it fires an internal **assert/abort** (`ode/src/joints/hinge2.cpp:238`). Steering ⊥ rolling is the natural
  choice; **guard user-supplied axes** before creating the joint. (Field-verified: parallel axes hard-abort, exit 134.)

## Never invent

- `dJointSetHinge2Axis` (singular) — the setter is `dJointSetHinge2Axes` (plural, both axes), include/ode/objects.h:2084.
- `dJointAddHinge2Torque` (singular) — the function is `dJointAddHinge2Torques` taking two torques, include/ode/objects.h:2118.
- `dParamSuspensionVel` / `dParamWheelSpeed` — no such params; use `dParamSuspensionERP/CFM` and `dParamVel2`.
- `dJointSetHinge2Suspension` — suspension is set via `dJointSetHinge2Param(j, dParamSuspensionERP/CFM, ...)`, not a dedicated setter.

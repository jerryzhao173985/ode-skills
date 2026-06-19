# Joint: PR (Prismatic + Rotoide)  (`dJointCreatePR`)

> Source of truth: include/ode/objects.h:1741. Cited by file:line; do not invent symbols.

Cross-link: see the joints overview at `references/joints.md` for the dParam family, joint lifecycle, and contact joints.

## What it is

The PR joint is a 2-DOF joint combining a **Prismatic** articulation (slide along axis 1) with a **Rotoide** articulation (rotate about axis 2). Axis 1 is the prismatic (sliding) axis; axis 2 is the rotoide (hinge) axis (include/ode/objects.h:1741; axis roles at include/ode/objects.h:2240,2246).

## Create & attach

```c
dJointID j = dJointCreatePR (world, 0);
dJointAttach (j, body1, body2);
dJointSetPRAnchor (j, ax, ay, az);     // world-space anchor
dJointSetPRAxis1  (j, 1, 0, 0);        // prismatic (slide) axis
dJointSetPRAxis2  (j, 0, 0, 1);        // rotoide (rotation) axis
```
Cited: `dJointCreatePR` ode/demo/demo_jointPR.cpp:380; `dJointAttach` ode/demo/demo_jointPR.cpp:381; `dJointSetPRAnchor` ode/demo/demo_jointPR.cpp:385; `dJointSetPRAxis1`/`dJointSetPRAxis2` ode/demo/demo_jointPR.cpp:412-413.

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetPRAnchor` | `void dJointSetPRAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2237 | Set the anchor in world coordinates. |
| `dJointSetPRAxis1` | `void dJointSetPRAxis1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2243 | Set the axis for the **prismatic** articulation. |
| `dJointSetPRAxis2` | `void dJointSetPRAxis2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2249 | Set the axis for the **rotoide** articulation. |
| `dJointSetPRParam` | `void dJointSetPRParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2257 | Set a param; `parameterX` with X=2 addresses the rotoide articulation. |
| `dJointAddPRTorque` | `void dJointAddPRTorque (dJointID j, dReal torque)` | include/ode/objects.h:2267 | Apply torque about the rotoide axis. |
| `dJointGetPRAnchor` | `void dJointGetPRAnchor (dJointID, dVector3 result)` | include/ode/objects.h:2804 | Get the anchor (point on body 1). |
| `dJointGetPRPosition` | `dReal dJointGetPRPosition (dJointID)` | include/ode/objects.h:2817 | Get the prismatic extension; zero is the config when the axis was set. |
| `dJointGetPRPositionRate` | `dReal dJointGetPRPositionRate (dJointID)` | include/ode/objects.h:2824 | Get the prismatic position's time derivative. |
| `dJointGetPRAngle` | `dReal dJointGetPRAngle (dJointID)` | include/ode/objects.h:2834 | Get the rotoide angle (twist between the 2 bodies). |
| `dJointGetPRAngleRate` | `dReal dJointGetPRAngleRate (dJointID)` | include/ode/objects.h:2841 | Get the rotoide angle's time derivative. |
| `dJointGetPRAxis1` | `void dJointGetPRAxis1 (dJointID, dVector3 result)` | include/ode/objects.h:2848 | Get the prismatic axis. |
| `dJointGetPRAxis2` | `void dJointGetPRAxis2 (dJointID, dVector3 result)` | include/ode/objects.h:2854 | Get the rotoide axis. |
| `dJointGetPRParam` | `dReal dJointGetPRParam (dJointID, int parameter)` | include/ode/objects.h:2860 | Read back a param value. |

## Parameters / conventions

- **dParam family** applies per articulation (include/ode/common.h:441-451). The **base** params (`dParamLoStop`, `dParamVel`, `dParamFMax`, ...) address the prismatic axis; the **`2`-suffixed** forms (`dParamLoStop2`, `dParamVel2`, ...) address the rotoide axis. The header states `parameterX where X equal 2 refer to parameter for the rotoide articulation` (include/ode/objects.h:2255), and the suffix arithmetic is `(dParamGroup2 | dParamFMax) == dParamFMax2` (include/ode/common.h:460-463).
- No suspension params (hinge-2 only — include/ode/common.h:453).
- **Axis convention:** axis 1 = prismatic/slide, axis 2 = rotoide/rotation; both vectors are in world coordinates.

## Pitfalls

- A motor on either articulation does nothing unless its `dParamFMax`/`dParamFMax2` is positive (inventory mistake; include/ode/common.h:446).
- Use the **`2`-suffixed** params for the rotoide stops/motor; passing the base form addresses the prismatic axis only (include/ode/objects.h:2255).
- `dJointGetPRPosition` (prismatic extension) and `dJointGetPRAngle` (rotoide twist) are distinct readouts — both zero at the moment the axis is set (include/ode/objects.h:2809,2830).
- Forgetting `dJointAttach` leaves the joint inert (include/ode/objects.h:1866).

## Never invent

- `dJointSetPRAxis` (no number) / `dJointSetPRAxis3` — the PR has exactly axis1 (prismatic) and axis2 (rotoide): include/ode/objects.h:2243,2249.
- `dJointAddPRForce` — only `dJointAddPRTorque` (about the rotoide axis) exists: include/ode/objects.h:2267.
- `dJointGetPRPosition2` — there is one prismatic readout (`dJointGetPRPosition`) and one angular readout (`dJointGetPRAngle`): include/ode/objects.h:2817,2834.

# Joint: PU (Prismatic + Universal)  (`dJointCreatePU`)

> Source of truth: include/ode/objects.h:1749. Cited by file:line; do not invent symbols.

Cross-link: see the joints overview at `references/joints.md` for the dParam family, joint lifecycle, and contact joints.

## What it is

The PU joint is a 3-DOF joint combining a **Universal** articulation (two rotation axes, axis 1 and axis 2) with a **Prismatic** articulation (slide along axis 3 / "axis P"). So: two hinge rotations plus one linear slide (include/ode/objects.h:1749; axis roles at include/ode/objects.h:2316,2322,2328).

## Create & attach

```c
dJointID j = dJointCreatePU (world, 0);
dJointAttach (j, body1, body2);
dJointSetPUAnchor (j, 0, 0, 2.5);    // world-space anchor
dJointSetPUAxis1  (j, 1, 0, 0);      // universal axis 1
dJointSetPUAxis2  (j, 0, 1, 0);      // universal axis 2
dJointSetPUAxis3  (j, 0, 0, 1);      // prismatic axis (a.k.a. axisP)
```
Cited: `dJointCreatePU` include/ode/objects.h:1749; `dJointAttach` include/ode/objects.h:1873; `dJointSetPUAnchor` include/ode/objects.h:2274 (used in ode/demo/demo_jointPU.cpp:722); `dJointSetPUAxis1`/`Axis2`/`Axis3` include/ode/objects.h:2319,2325,2331.

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetPUAnchor` | `void dJointSetPUAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2274 | Set the anchor in world coordinates. |
| `dJointSetPUAnchorOffset` | `void dJointSetPUAnchorOffset (dJointID, dReal x, dReal y, dReal z, dReal dx, dReal dy, dReal dz)` | include/ode/objects.h:2312 | Set the anchor as if the bodies were already `[dx,dy,dz]` apart (presets the position). |
| `dJointSetPUAxis1` | `void dJointSetPUAxis1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2319 | Set the first axis of the universal articulation. |
| `dJointSetPUAxis2` | `void dJointSetPUAxis2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2325 | Set the second axis of the universal articulation. |
| `dJointSetPUAxis3` | `void dJointSetPUAxis3 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2331 | Set the prismatic axis. |
| `dJointSetPUAxisP` | `void dJointSetPUAxisP (dJointID id, dReal x, dReal y, dReal z)` | include/ode/objects.h:2339 | Convenience alias — same as `dJointSetPUAxis3`. |
| `dJointSetPUParam` | `void dJointSetPUParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2352 | Set a param; X=2 → universal axis 2, X=3 → prismatic. |
| `dJointAddPUTorques` | `void dJointAddPUTorques (dJointID joint, dReal torque1, dReal torque2)` | include/ode/objects.h:2364 | Apply torque1 about universal axis 1, torque2 about axis 2. |
| `dJointGetPUAnchor` | `void dJointGetPUAnchor (dJointID, dVector3 result)` | include/ode/objects.h:2870 | Get the anchor. |
| `dJointGetPUPosition` | `dReal dJointGetPUPosition (dJointID)` | include/ode/objects.h:2883 | Get the prismatic extension. |
| `dJointGetPUPositionRate` | `dReal dJointGetPUPositionRate (dJointID)` | include/ode/objects.h:2890 | Get the prismatic position's time derivative. |
| `dJointGetPUAxis1` | `void dJointGetPUAxis1 (dJointID, dVector3 result)` | include/ode/objects.h:2896 | Get universal axis 1. |
| `dJointGetPUAxis2` | `void dJointGetPUAxis2 (dJointID, dVector3 result)` | include/ode/objects.h:2902 | Get universal axis 2. |
| `dJointGetPUAxis3` | `void dJointGetPUAxis3 (dJointID, dVector3 result)` | include/ode/objects.h:2908 | Get the prismatic axis. |
| `dJointGetPUAxisP` | `void dJointGetPUAxisP (dJointID id, dVector3 result)` | include/ode/objects.h:2917 | Convenience alias — same as `dJointGetPUAxis3`. |
| `dJointGetPUAngles` | `void dJointGetPUAngles (dJointID, dReal *angle1, dReal *angle2)` | include/ode/objects.h:2933 | Get both universal angles at once. |
| `dJointGetPUAngle1` | `dReal dJointGetPUAngle1 (dJointID)` | include/ode/objects.h:2939 | Get universal angle 1. |
| `dJointGetPUAngle1Rate` | `dReal dJointGetPUAngle1Rate (dJointID)` | include/ode/objects.h:2946 | Get universal angle 1 rate. |
| `dJointGetPUAngle2` | `dReal dJointGetPUAngle2 (dJointID)` | include/ode/objects.h:2953 | Get universal angle 2. |
| `dJointGetPUAngle2Rate` | `dReal dJointGetPUAngle2Rate (dJointID)` | include/ode/objects.h:2960 | Get universal angle 2 rate. |
| `dJointGetPUParam` | `dReal dJointGetPUParam (dJointID, int parameter)` | include/ode/objects.h:2966 | Read back a param value. |

Deprecated (do not use): `dJointSetPUAnchorDelta` (include/ode/objects.h:2280, `ODE_API_DEPRECATED`).

## Parameters / conventions

- **dParam family** maps per articulation (include/ode/common.h:441-451): **base** params → universal axis 1, **`2`-suffixed** → universal axis 2, **`3`-suffixed** → prismatic. The header states `parameterX where X equal 2 refer to parameter for second axis of the universal articulation` and `X equal 3 refer to parameter for prismatic articulation` (include/ode/objects.h:2347-2350); suffix arithmetic is `(dParamGroup2 | dParamFMax) == dParamFMax2` (include/ode/common.h:460-463).
- No suspension params (hinge-2 only — include/ode/common.h:453).
- **Axis convention:** axis1 + axis2 = universal (rotation), axis3 = axisP = prismatic (slide); all in world coordinates. `dJointSetPUAxisP` is just a convenience name for `dJointSetPUAxis3` (include/ode/objects.h:2336-2337).

## Pitfalls

- A motor on any articulation does nothing unless its `dParamFMax`/`dParamFMax2`/`dParamFMax3` is positive (inventory mistake; include/ode/common.h:446).
- Address the correct articulation by suffix: base→universal-1, `2`→universal-2, `3`→prismatic; passing the base form silently controls only universal axis 1 (include/ode/objects.h:2347-2350).
- `dJointAddPUTorques` applies torque only about the **two universal axes**; the prismatic DOF is driven via params/forces, not by this call (include/ode/objects.h:2355-2364).
- Forgetting `dJointAttach` leaves the joint inert (include/ode/objects.h:1866).

## Never invent

- `dJointSetPUAxis4` — the PU has axis1, axis2, axis3 (=axisP) only: include/ode/objects.h:2319,2325,2331,2339.
- `dJointAddPUTorque` (singular) / `dJointAddPUForce` — the function is `dJointAddPUTorques` (two torques): include/ode/objects.h:2364.
- `dJointGetPUAngle3` — the prismatic DOF reports a **position** (`dJointGetPUPosition`), not an angle: include/ode/objects.h:2883.

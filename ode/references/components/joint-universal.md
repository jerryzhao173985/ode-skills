# Joint: Universal  (`dJointCreateUniversal`)

> Source of truth: include/ode/objects.h:1733. Cited by file:line; do not invent symbols.

## What it is

A universal joint allows **2 rotational DOF** about two perpendicular axes through a common anchor (a U-joint / Cardan joint). It is like a ball joint with one rotational DOF removed: it removes 4 DOF (3 translational + 1 rotational), leaving rotation about `axis1` and `axis2` free.

## Create & attach

```c
dJointID joint = dJointCreateUniversal (world, 0);
dJointAttach (joint, body1, body2);                /* REQUIRED; 0 for a body => static env */
dJointSetUniversalAnchor (joint, x, y, z);         /* anchor in WORLD coords */
dJointSetUniversalAxis1  (joint, a1x, a1y, a1z);   /* first axis, WORLD coords */
dJointSetUniversalAxis2  (joint, a2x, a2y, a2z);   /* second axis, perpendicular to axis1 */
```

Verbatim shape from `ode/demo/demo_joints.cpp:437-441`:

```c
joint = dJointCreateUniversal (world,0);
dJointAttach (joint,body[0],body[1]);
dJointSetUniversalAnchor (joint,0,0,1);
dJointSetUniversalAxis1 (joint, 1, -1, 1.41421356);
dJointSetUniversalAxis2 (joint, 1, -1, -1.41421356);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetUniversalAnchor` | `void dJointSetUniversalAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2124 | Set the universal-joint anchor in world coordinates. |
| `dJointSetUniversalAxis1` | `void dJointSetUniversalAxis1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2130 | Set the first rotation axis in world coordinates. |
| `dJointSetUniversalAxis2` | `void dJointSetUniversalAxis2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2175 | Set the second rotation axis in world coordinates. |
| `dJointSetUniversalParam` | `void dJointSetUniversalParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2222 | Set a universal-joint parameter via a `dParam*` constant. |
| `dJointGetUniversalAnchor` | `void dJointGetUniversalAnchor (dJointID, dVector3 result)` | include/ode/objects.h:2723 | Read the anchor as seen on body1. |
| `dJointGetUniversalAxis1` | `void dJointGetUniversalAxis1 (dJointID, dVector3 result)` | include/ode/objects.h:2744 | Read the first axis. |
| `dJointGetUniversalAxis2` | `void dJointGetUniversalAxis2 (dJointID, dVector3 result)` | include/ode/objects.h:2750 | Read the second axis. |
| `dJointGetUniversalAngle1` | `dReal dJointGetUniversalAngle1 (dJointID)` | include/ode/objects.h:2776 | Read the angle about axis1. |
| `dJointGetUniversalAngle2` | `dReal dJointGetUniversalAngle2 (dJointID)` | include/ode/objects.h:2782 | Read the angle about axis2. |

## Parameters / conventions

- **Anchor + axis1 + axis2**: all in world coordinates, set after attach. The two axes should be perpendicular; each axis carries its own angle (`dJointGetUniversalAngle1` / `...Angle2`).
- **Per-axis dParam addressing**: axis1 uses the **base** `dParam*` names; axis2 uses the **+1 suffixed** forms (e.g. `dParamLoStop2`, `dParamVel2`) or `base + dParamGroup`. The `dParamGroup` stride is `0x100` (`include/ode/common.h:494`), and second/third-axis parameters use the suffixed names per `include/ode/objects.h:1678`.
- **Stops/motor**: same `dParam*` family as the hinge — `dParamLoStop`/`dParamHiStop` (limits), `dParamVel` + `dParamFMax` (motor; no torque unless `dParamFMax > 0`) (`include/ode/common.h:441-446`), applied independently to each axis via `dJointSetUniversalParam`.

## Pitfalls

- Setting only the base `dParam*` constant addresses **axis1 only**; use the suffixed `...2` form (or `+ dParamGroup`) for axis2 (`include/ode/objects.h:1678`, mistakes block).
- A motor on either axis does nothing unless that axis's `dParamFMax` is positive (`include/ode/common.h:446`).
- Forgetting `dJointAttach` leaves the joint inert (`include/ode/objects.h:1688`); 0 for a body anchors to the static environment (`include/ode/objects.h:1866`).
- **The two axes must be linearly independent — parallel axes ABORT ODE.** A universal joint derives its frame from the **cross product** of the two axes; parallel axes give a zero-length cross product, so ODE's `dNormalize3` fails and **aborts the process** (`ode/src/joints/universal.cpp:327-328`). Keep them well separated (perpendicular is ideal) and **guard user-supplied axes before creating the joint**. (Field-verified: a Universal/Hinge2 built with parallel axes hard-aborts, exit 134.)

## Never invent

- `dJointSetUniversalAxes` — set the two axes separately with `dJointSetUniversalAxis1` and `dJointSetUniversalAxis2` (the combined `dJointSetHinge2Axes` form is hinge-2 only).
- `dParamMaxForce` / `dParamMotorVel` — use `dParamFMax` / `dParamVel` (with the `...2` suffix for axis2).
- `dJointGetUniversalForce` — read forces via `dJointFeedback` set with `dJointSetFeedback`.

See also the joints overview: `references/joints.md`.

# Joint: Hinge (revolute)  (`dJointCreateHinge`)

> Source of truth: include/ode/objects.h:1701. Cited by file:line; do not invent symbols.

## What it is

A hinge (revolute) joint allows **1 rotational DOF**: the two bodies rotate about a single shared axis through a fixed anchor, like a door hinge. It removes 5 DOF (3 translational + 2 rotational), leaving rotation about the hinge axis free.

## Create & attach

```c
dJointID joint = dJointCreateHinge (world, 0);
dJointAttach (joint, body1, body2);                /* REQUIRED; 0 for a body => static env */
dJointSetHingeAnchor (joint, x, y, z);             /* anchor in WORLD coords */
dJointSetHingeAxis   (joint, ax, ay, az);          /* axis in WORLD coords */
```

Verbatim shape from `ode/demo/demo_feedback.cpp:251-256` (powered hinge):

```c
hinges[i] = dJointCreateHinge (world,0);
dJointAttach (hinges[i], segbodies[i],segbodies[i+1]);
dJointSetHingeAnchor (hinges[i], i + 0.5 - SEGMCNT/2.0, 0, 5);
dJointSetHingeAxis (hinges[i], 0,1,0);
dJointSetHingeParam (hinges[i],dParamFMax,  8000.0);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetHingeAnchor` | `void dJointSetHingeAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1983 | Set the hinge anchor in world coordinates. |
| `dJointSetHingeAxis` | `void dJointSetHingeAxis (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1991 | Set the hinge rotation axis; resets the angle "zero". |
| `dJointSetHingeParam` | `void dJointSetHingeParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2027 | Set a hinge limit/motor parameter via a `dParam*` constant. |
| `dJointAddHingeTorque` | `void dJointAddHingeTorque (dJointID joint, dReal torque)` | include/ode/objects.h:2037 | Apply a torque about the hinge axis (wrapper over `dBodyAddTorque`). |
| `dJointGetHingeAnchor` | `void dJointGetHingeAnchor (dJointID, dVector3 result)` | include/ode/objects.h:2582 | Read the anchor as seen on body1. |
| `dJointGetHingeAnchor2` | `void dJointGetHingeAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:2592 | Read the anchor on body2; differs from `dJointGetHingeAnchor` when the joint comes apart. |
| `dJointGetHingeAxis` | `void dJointGetHingeAxis (dJointID, dVector3 result)` | include/ode/objects.h:2598 | Read the current hinge axis. |
| `dJointGetHingeAngle` | `dReal dJointGetHingeAngle (dJointID)` | include/ode/objects.h:2618 | Current angle (range -pi..pi) relative to the zero set when the axis/anchor was assigned. |
| `dJointGetHingeParam` | `dReal dJointGetHingeParam (dJointID, int parameter)` | include/ode/objects.h:2604 | Read back a hinge `dParam*` value. |

## Parameters / conventions

- **Anchor + axis**: both in world coordinates, set after attach. The angle is measured between the two bodies about the axis; setting the anchor or axis re-examines the current pose and makes that the **zero angle** (`include/ode/objects.h:2018`, `include/ode/objects.h:2618`).
- **Stops** (apply via `dJointSetHingeParam`): `dParamLoStop` (include/ode/common.h:441), `dParamHiStop` (include/ode/common.h:442), plus `dParamBounce` / `dParamStopERP` / `dParamStopCFM` for stop behavior (include/ode/common.h:448-451).
- **Motor**: `dParamVel` (desired angular velocity, include/ode/common.h:443) together with `dParamFMax` (max torque, include/ode/common.h:446). The motor produces **no force unless `dParamFMax > 0`**.

## Pitfalls

- Setting only `dParamVel` with `dParamFMax` left at 0 yields no motor torque; always set both (`include/ode/objects.h` motor inventory; mistakes block).
- `dJointSetHingeAnchor` or `dJointSetHingeAxis` resets the angle "zero" reference — call them before reading `dJointGetHingeAngle` if you depend on a baseline (`include/ode/objects.h:2018`).
- Forgetting `dJointAttach` leaves the joint inert (`include/ode/objects.h:1688`); passing 0 for one body anchors to the static environment (`include/ode/objects.h:1866`).
- An unimplemented/inapplicable parameter is silently ignored (`include/ode/objects.h:1678`).

## Never invent

- `dParamMaxForce` / `dParamMotorVel` / `dParamLowStop` / `dParamHighStop` — correct names are `dParamFMax`, `dParamVel`, `dParamLoStop`, `dParamHiStop`.
- `dJointSetHingeMotor` / `dJointSetHingeLimit` — use `dJointSetHingeParam` with the `dParam*` constants.
- `dJointGetHingeForce` / `dJointGetHingeTorque` — read forces via `dJointFeedback` registered with `dJointSetFeedback`.

See also the joints overview: `references/joints.md`.

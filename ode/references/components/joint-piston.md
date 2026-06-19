# Joint: Piston  (`dJointCreatePiston`)

> Source of truth: include/ode/objects.h:1758. Cited by file:line; do not invent symbols.

Cross-link: see the joints overview at `references/joints.md` for the dParam family, joint lifecycle, and contact joints.

## What it is

The piston is a 2-DOF joint: the two bodies may **slide** along a single axis (prismatic, like a slider) **and rotate** about that same axis. It combines slider-style linear travel with free spin around the axis (include/ode/objects.h:1758).

## Create & attach

```c
dJointID j = dJointCreatePiston (world, 0);
dJointAttach (j, body1, body2);                 // one body may be 0 (static env)
dJointSetPistonAnchor (j, ax, ay, az);          // world-space anchor
dJointSetPistonAxis   (j, 1, 0, 0);             // slide/spin axis, world coords
```
Cited: `dJointCreatePiston` include/ode/objects.h:1758; `dJointAttach` include/ode/objects.h:1873; `dJointSetPistonAnchor` include/ode/objects.h:2373 (used in ode/demo/demo_piston.cpp:296); `dJointSetPistonAxis` include/ode/objects.h:2411. (`dJointAttach(j, body, 0)` / `dJointAttach(j, 0, body)` are both valid — ode/demo/demo_piston.cpp:399,402.)

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetPistonAnchor` | `void dJointSetPistonAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2373 | Set the joint anchor in world coordinates. |
| `dJointSetPistonAnchorOffset` | `void dJointSetPistonAnchorOffset(dJointID j, dReal x, dReal y, dReal z, dReal dx, dReal dy, dReal dz)` | include/ode/objects.h:2404 | Set the anchor as if the bodies were already `[dx,dy,dz]` apart (presets the position). |
| `dJointSetPistonAxis` | `void dJointSetPistonAxis (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2411 | Set the slide/spin axis in world coordinates. |
| `dJointSetPistonParam` | `void dJointSetPistonParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2432 | Set a limit/motor param via a dParam* constant. |
| `dJointAddPistonForce` | `void dJointAddPistonForce (dJointID joint, dReal force)` | include/ode/objects.h:2442 | Apply a force along the piston axis (wrapper for dBodyAddForce). |
| `dJointGetPistonPosition` | `dReal dJointGetPistonPosition (dJointID)` | include/ode/objects.h:2979 | Get the linear position; zero is the configuration when the axis was set. |
| `dJointGetPistonPositionRate` | `dReal dJointGetPistonPositionRate (dJointID)` | include/ode/objects.h:2985 | Get the linear position's time derivative. |
| `dJointGetPistonAngle` | `dReal dJointGetPistonAngle (dJointID)` | include/ode/objects.h:2994 | Get the rotation angle about the axis. |
| `dJointGetPistonAngleRate` | `dReal dJointGetPistonAngleRate (dJointID)` | include/ode/objects.h:3000 | Get the angular rate about the axis. |
| `dJointGetPistonAnchor` | `void dJointGetPistonAnchor (dJointID, dVector3 result)` | include/ode/objects.h:3012 | Get the anchor as seen on body 1. |
| `dJointGetPistonAnchor2` | `void dJointGetPistonAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:3027 | Get the anchor as seen on body 2 (to see how far the joint has come apart). |
| `dJointGetPistonAxis` | `void dJointGetPistonAxis (dJointID, dVector3 result)` | include/ode/objects.h:3033 | Get the axis. |
| `dJointGetPistonParam` | `dReal dJointGetPistonParam (dJointID, int parameter)` | include/ode/objects.h:3039 | Read back a param value. |

Deprecated (do not use): `dJointSetPistonAxisDelta` (include/ode/objects.h:2426, `ODE_API_DEPRECATED`).

## Parameters / conventions

- **dParam family** applies on the single axis (include/ode/common.h:441-451): `dParamLoStop`/`dParamHiStop` bound the linear travel; `dParamVel`/`dParamFMax`/`dParamBounce`/`dParamFudgeFactor`/`dParamStopERP`/`dParamStopCFM` tune the motor and stops.
- The piston has **no suspension** params (those are hinge-2 only — include/ode/common.h:453).
- **Axis convention:** the axis vector is the world-space line the bodies slide along and spin around; the rotation DOF shares that same axis.
- **Get-position semantics:** `dJointGetPistonPosition`/`...Rate` report linear extension; `dJointGetPistonAngle`/`...Rate` report the twist about the axis. Zero is set when the axis is assigned (include/ode/objects.h:2975-2976).
- **Anchor offset:** use `dJointSetPistonAnchorOffset` to preset a nonzero starting position; the header's worked example shows `dJointGetPistonPosition(jId) == offset` afterwards (include/ode/objects.h:2384-2392).

## Pitfalls

- A piston motor does nothing unless `dParamFMax > 0`; `dParamVel` alone produces no force (inventory mistake; include/ode/common.h:446).
- `dJointGetPistonPosition` and `dJointGetPistonAngle` are two different readouts — the prismatic extension vs. the spin angle; do not conflate them (include/ode/objects.h:2979 vs 2994).
- Forgetting `dJointAttach` leaves the joint inert; passing 0 for one body anchors to the static environment, 0 for both leaves it in limbo (include/ode/objects.h:1866).

## Never invent

- `dJointSetPistonAxis2` — the piston has a single axis (`dJointSetPistonAxis`, include/ode/objects.h:2411).
- `dJointAddPistonTorque` — only `dJointAddPistonForce` exists (include/ode/objects.h:2442); spin about the axis is free, not motor-torqued via a dedicated call.
- `dJointGetPistonPosition2` / `dJointSetPistonParam2` — the piston is single-axis; there are no `2`-suffixed piston params.

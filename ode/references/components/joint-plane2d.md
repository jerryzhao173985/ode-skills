# Joint: Plane-2D  (`dJointCreatePlane2D`)

> Source of truth: include/ode/objects.h:1792. Cited by file:line; do not invent symbols.

Cross-link: see the joints overview at `references/joints.md` for the dParam family, joint lifecycle, and contact joints.

## What it is

The plane-2D joint confines a single body to the **z = const plane**, leaving it 3 DOF: translation in X, translation in Y, and rotation about Z. It is attached to one body (the other side is the static environment) (include/ode/objects.h:1792 / inventory summary "confines a body to the z=const plane").

## Create & attach

```c
dJointID j = dJointCreatePlane2D (world, 0);
dJointAttach (j, body, 0);                 // body1=the body, body2=0 (static env)
dJointSetPlane2DXParam (j, dParamFMax, 10);  // optional: motor along world X
dJointSetPlane2DYParam (j, dParamFMax, 10);  // optional: motor along world Y
```
Cited: `dJointCreatePlane2D`/`dJointAttach` ode/demo/demo_plane2d.cpp:263-264; `dJointSetPlane2DXParam`/`...YParam` ode/demo/demo_plane2d.cpp:267-268.

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetPlane2DXParam` | `void dJointSetPlane2DXParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2536 | Set a dParam* for the world-X translational DOF. |
| `dJointSetPlane2DYParam` | `void dJointSetPlane2DYParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2542 | Set a dParam* for the world-Y translational DOF. |
| `dJointSetPlane2DAngleParam` | `void dJointSetPlane2DAngleParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2547 | Set a dParam* for the rotation-about-Z DOF. |

This joint has **no anchor/axis setters and no getters** in the public header — the plane (z=const) and the three DOF are implicit; you only tune the per-DOF dParam motors via the three `*Param` functions above (include/ode/objects.h:2536-2547).

## Parameters / conventions

- **dParam family** (include/ode/common.h:441-451): each DOF is driven independently — `dJointSetPlane2DXParam`, `...YParam`, `...AngleParam` — each taking base dParam constants like `dParamFMax`, `dParamVel`, `dParamLoStop`, `dParamHiStop`. The demo sets a velocity servo per axis: `dJointSetPlane2DXParam(id, dParamVel, k*(target_x - curr_x))` (ode/demo/demo_plane2d.cpp:111-112) with `dParamFMax` to enable it (ode/demo/demo_plane2d.cpp:267-268).
- **No `2`/`3`-suffixed params here** — the three DOF are addressed by three **separate functions**, not by suffixed constants.
- No suspension params (hinge-2 only — include/ode/common.h:453).
- **Convention:** X and Y are world-axis translations; Angle is rotation about world Z. There is no `dJointSetPlane2D*Axis` because the constrained plane is fixed at z=const.

## Pitfalls

- A per-DOF motor does nothing unless its `dParamFMax > 0`; the demo sets `dParamFMax` once and then drives `dParamVel` each step (ode/demo/demo_plane2d.cpp:267-268 then 111-112; FMax requirement include/ode/common.h:446).
- Attach to **one** body with body2=0; the plane-2D constraint is meant to pin a body to the z-plane relative to the static environment (ode/demo/demo_plane2d.cpp:264).
- Use the three dedicated `*Param` setters — there is no generic `dJointSetPlane2DParam` taking an axis index (include/ode/objects.h:2536-2547).
- Forgetting `dJointAttach` leaves the joint inert (include/ode/objects.h:1866).

## Never invent

- `dJointSetPlane2DParam` (single generic setter) — there are exactly three: `...XParam`, `...YParam`, `...AngleParam` (include/ode/objects.h:2536,2542,2547).
- `dJointSetPlane2DAnchor` / `dJointSetPlane2DAxis` — the plane-2D joint has no anchor or axis setters in the public header.
- `dJointGetPlane2DPosition` / `dJointGetPlane2DAngle` — no plane-2D getters exist in the public header; read pose via the body (`dBodyGetPosition`/`dBodyGetRotation`).
- `dParamVelZ` / a Z-translation param — Z is constrained out; only X, Y, and Angle DOF are exposed.

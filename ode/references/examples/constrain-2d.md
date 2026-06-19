# Example: constrain to 2D — the Plane2D joint

> Source: wiki **Constrain Objects to 2D** (`ode.org/wiki/index.php/Constrain_Objects_to_2D`) + `ode/demo/demo_plane2d.cpp`, grounded against `include/ode/objects.h`. How to pin bodies to the x-y plane for a 2D game.

## What it builds
Bodies confined to the z = const plane (translation in x/y, rotation only about z), optionally driven by the joint's built-in 2D motors, in a walled arena.

## Components composed
A **Plane2D Joint** per body (`dJointCreatePlane2D`) attached to the **static world** (`dJointAttach(j, body, 0)`); it is a built-in 2D LMotor (x,y) + 1D AMotor (z). Plus 4 wall **Geoms** (`dCreatePlane`).

## Setup (from `demo_plane2d.cpp`)
```c
dJointID pj = dJointCreatePlane2D(world, 0);   // objects.h:1792
dJointAttach(pj, body, 0);                      // objects.h:1873 — 2nd body 0 = static world
// only a body you intend to DRIVE needs a motor force budget:
dJointSetPlane2DXParam(pj, dParamFMax, 10);     // objects.h:2536
dJointSetPlane2DYParam(pj, dParamFMax, 10);     // objects.h:2542
```

## Driving a 2D body to a target (proportional, every step)
```c
dJointSetPlane2DXParam(pj, dParamVel, 1*(target_x - body.getPosition()[0]));
dJointSetPlane2DYParam(pj, dParamVel, 1*(target_y - body.getPosition()[1]));
// dParamFMax must be > 0 (set once at init) or dParamVel does nothing.
```
(`dJointSetPlane2DAngleParam` `objects.h:2547` drives the z-rotation motor. Motor *limits* aren't supported for Plane2D — they don't apply.)

## The mandatory drift correction (the #1 gotcha)
**Plane2D stops out-of-plane rotation but does NOT correct accumulated drift.** Every step, after `dWorldStep`, snap each constrained body's quaternion to a pure-z rotation and zero non-z angular velocity (`demo_plane2d.cpp:153-168`):
```c
const dReal *av = dBodyGetAngularVel(b);
const dReal *q  = dBodyGetQuaternion(b);
dReal nq[4] = { q[0], 0, 0, q[3] };             // kill x,y rotation components
dReal len = sqrt(nq[0]*nq[0] + nq[3]*nq[3]);
nq[0] /= len; nq[3] /= len;                      // renormalize
dBodySetQuaternion(b, nq);
dBodySetAngularVel(b, 0, 0, av[2]);              // objects.h:1077 — z-only spin
```

## Tokens (from the demo)
World `ERP=0.5`, `CFM=0.001`, gravity `(0,0,-1)`; the demo substeps 10× per frame at `dt/10` for stability. See `references/tokens.md`.

## Gotchas
- **Run the drift correction on every constrained body, every step** — or bodies slowly tilt out of plane.
- A Plane2D motor is **inert until `dParamFMax > 0`**.
- Attach with body2 = `0` (the world); a real second body changes the semantics.

## See also
`references/joints.md` (Plane2D + `dParam*`), `references/foundations/erp-cfm-friction.md`.

# Joint: Linear Motor (`dJointCreateLMotor`)

> Source of truth: include/ode/objects.h:1784. Cited by file:line; do not invent symbols.

## What it is

A linear motor (LMotor) drives and/or constrains the relative *linear* (translational) velocity of two bodies along 1-3 independently-controlled axes (DOF = number of axes set via `dJointSetLMotorNumAxes`, 0-3, where 0 effectively deactivates the joint). Cited by include/ode/objects.h:1784, include/ode/objects.h:2512, include/ode/objects.h:2508.

## Create & attach

```c
dJointID m = dJointCreateLMotor(world, 0);   /* group 0 = normal alloc */
dJointAttach(m, body1, body2);               /* required; 0 for a body = static env */
dJointSetLMotorNumAxes(m, 1);                /* 0..3 axes; 0 deactivates */
dJointSetLMotorAxis(m, 0, 0, 0, 0, 1);       /* axis 0, rel=0 (global), vector (0,0,1) */
dJointSetLMotorParam(m, dParamVel, 1.0);
dJointSetLMotorParam(m, dParamFMax, 50.0);   /* must be > 0 or motor does nothing */
```

Cited by include/ode/objects.h:1784, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:2512 (num axes), include/ode/objects.h:2525 (axis), include/ode/objects.h:2531 (param), include/ode/common.h:443 / include/ode/common.h:446 (dParamVel / dParamFMax).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetLMotorNumAxes` | `void dJointSetLMotorNumAxes (dJointID, int num)` | include/ode/objects.h:2512 | Set how many axes the LMotor controls (0-3; 0 deactivates). |
| `dJointSetLMotorAxis` | `void dJointSetLMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)` | include/ode/objects.h:2525 | Set axis `anum`; `rel` = 0 global / 1 body1 / 2 body2; vector always global. |
| `dJointSetLMotorParam` | `void dJointSetLMotorParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2531 | Set an LMotor parameter using a `dParam*` constant. |
| `dJointGetLMotorNumAxes` | `int dJointGetLMotorNumAxes (dJointID)` | include/ode/objects.h:3126 | Get the number of controlled axes. |
| `dJointGetLMotorAxis` | `void dJointGetLMotorAxis (dJointID, int anum, dVector3 result)` | include/ode/objects.h:3132 | Get axis `anum` into `result`. |
| `dJointGetLMotorParam` | `dReal dJointGetLMotorParam (dJointID, int parameter)` | include/ode/objects.h:3138 | Read back an LMotor `dParam*` value. |

## Parameters / conventions

- Motor params per axis: `dParamVel` (target *linear* velocity) and `dParamFMax` (max force). Cited by include/ode/common.h:443, include/ode/common.h:446.
- Axis 2/3 parameters use the suffixed forms (e.g. `dParamVel2`, `dParamFMax3`) or base + `dParamGroup`*n (`dParamGroup=0x100`). Cited by include/ode/objects.h:1678, include/ode/common.h:494.
- `dJointSetLMotorAxis` `rel`: 0 = global frame, 1 = anchored to first body, 2 = anchored to second body; the axis vector is always specified in global coordinates regardless of `rel`. Cited by include/ode/objects.h:2515.
- `num` can range from 0 (which effectively deactivates the joint) to 3. Cited by include/ode/objects.h:2508.

## Pitfalls

- A motor produces no force unless `dParamFMax > 0`; setting only `dParamVel` does nothing. Cited by include/ode/common.h:446.
- Setting `dJointSetLMotorNumAxes(m, 0)` effectively deactivates the joint. Cited by include/ode/objects.h:2508.
- The axis vector is in global coordinates even when `rel` anchors it to a body. Cited by include/ode/objects.h:2519.
- Inert until `dJointAttach` is called. Cited by include/ode/objects.h:1688.

## Never invent

- `dJointSetMotorVelocity` / `dJointSetMaxForce` — use `dJointSetLMotorParam(j, dParamVel / dParamFMax, ...)`. Cited by `03-joints.json` never_invent.
- `dJointGetLMotorPosition` — no such getter exists in include/ode/objects.h (LMotor exposes only NumAxes/Axis/Param getters). Cited by include/ode/objects.h:3126.
- `dJointAddLMotorForces` — no such function; apply forces via `dBodyAddForce`. Cited by include/ode/objects.h:3138.

Cross-link: references/joints.md (joints overview).

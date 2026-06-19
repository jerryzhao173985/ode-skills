# Joint: Angular Motor (`dJointCreateAMotor`)

> Source of truth: include/ode/objects.h:1776. Cited by file:line; do not invent symbols.

## What it is

An angular motor (AMotor) drives and/or constrains the relative angular motion of two bodies about 1-3 independently-controlled axes (DOF = number of axes set via `dJointSetAMotorNumAxes`, 0-3). It has two modes: `dAMotorUser` (you supply the axes and, if using stops, the current angles) and `dAMotorEuler` (ODE computes Euler angles automatically and forces 3 axes). Cited by include/ode/objects.h:1776, include/ode/objects.h:2465, include/ode/objects.h:3047.

## Create & attach

```c
dJointID m = dJointCreateAMotor(world, 0);   /* group 0 = normal alloc */
dJointAttach(m, body1, body2);               /* required; 0 for a body = static env */
dJointSetAMotorMode(m, dAMotorUser);         /* dAMotorUser or dAMotorEuler */
dJointSetAMotorNumAxes(m, 1);                /* 0..3 axes */
dJointSetAMotorAxis(m, 0, 1, 0, 0, 1);       /* axis 0, rel=1 (body1), vector (0,0,1) */
dJointSetAMotorParam(m, dParamVel, 2.0);
dJointSetAMotorParam(m, dParamFMax, 100.0);  /* must be > 0 or motor does nothing */
```

Cited by include/ode/objects.h:1776, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:2495 (mode), include/ode/objects.h:2465 (num axes), include/ode/objects.h:2471 (axis), include/ode/objects.h:2489 (param), include/ode/common.h:443 / include/ode/common.h:446 (dParamVel / dParamFMax).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetAMotorNumAxes` | `void dJointSetAMotorNumAxes (dJointID, int num)` | include/ode/objects.h:2465 | Set how many axes the AMotor controls (0-3). |
| `dJointSetAMotorAxis` | `void dJointSetAMotorAxis (dJointID, int anum, int rel, dReal x, dReal y, dReal z)` | include/ode/objects.h:2471 | Set AMotor axis `anum` with relative-orientation mode `rel`. |
| `dJointSetAMotorAngle` | `void dJointSetAMotorAngle (dJointID, int anum, dReal angle)` | include/ode/objects.h:2483 | Tell the AMotor the current angle along `anum`; only valid in `dAMotorUser` mode. |
| `dJointSetAMotorParam` | `void dJointSetAMotorParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2489 | Set an AMotor parameter; axes 2/3 use suffixed `dParam*` forms. |
| `dJointSetAMotorMode` | `void dJointSetAMotorMode (dJointID, int mode)` | include/ode/objects.h:2495 | Set mode (`dAMotorUser` or `dAMotorEuler`). |
| `dJointAddAMotorTorques` | `void dJointAddAMotorTorques (dJointID, dReal torque1, dReal torque2, dReal torque3)` | include/ode/objects.h:2505 | Apply torques about axes 0/1/2 (extra torques ignored if fewer axes). |
| `dJointGetAMotorNumAxes` | `int dJointGetAMotorNumAxes (dJointID)` | include/ode/objects.h:3050 | Get number of axes (auto-set to 3 in `dAMotorEuler`). |
| `dJointGetAMotorAxis` | `void dJointGetAMotorAxis (dJointID, int anum, dVector3 result)` | include/ode/objects.h:3061 | Get axis `anum` into `result`. |
| `dJointGetAMotorAxisRel` | `int dJointGetAMotorAxisRel (dJointID, int anum)` | include/ode/objects.h:3079 | Get the `rel` orientation mode of axis `anum`. |
| `dJointGetAMotorAngle` | `dReal dJointGetAMotorAngle (dJointID, int anum)` | include/ode/objects.h:3089 | Get angle along `anum` (set value in User mode; Euler angle in Euler mode). |
| `dJointGetAMotorAngleRate` | `dReal dJointGetAMotorAngleRate (dJointID, int anum)` | include/ode/objects.h:3099 | Get angle rate (always 0 in User mode). |
| `dJointGetAMotorParam` | `dReal dJointGetAMotorParam (dJointID, int parameter)` | include/ode/objects.h:3105 | Read back an AMotor `dParam*` value. |
| `dJointGetAMotorMode` | `int dJointGetAMotorMode (dJointID)` | include/ode/objects.h:3120 | Get current mode. |

## Parameters / conventions

- Motor params: `dParamVel` (target angular velocity) and `dParamFMax` (max torque) drive each axis; `dParamLoStop`/`dParamHiStop`/`dParamBounce`/`dParamStopERP`/`dParamStopCFM` define stops. Cited by include/ode/common.h:443, include/ode/common.h:446, include/ode/common.h:441, include/ode/common.h:442.
- Axis 2 and axis 3 parameters use the suffixed forms (e.g. `dParamVel2`, `dParamLoStop3`) or base + `dParamGroup`*n (`dParamGroup=0x100`). Cited by include/ode/objects.h:1678, include/ode/common.h:494.
- `dJointSetAMotorAxis` `rel`: 0 = global frame, 1 = anchored to first body, 2 = anchored to second body; the axis vector is always given in global coordinates. Cited by include/ode/objects.h:2515 (the LMotor-shared `rel` doc block).
- `dAMotorEuler` automatically sets 3 axes and computes Euler angles, so `dJointGetAMotorNumAxes` returns 3. Cited by include/ode/objects.h:3047, include/ode/objects.h:3112.

## Pitfalls

- A motor produces no torque unless `dParamFMax > 0`; setting only `dParamVel` does nothing. Cited by include/ode/common.h:446.
- `dJointSetAMotorAngle` should only be called in `dAMotorUser` mode — in Euler mode ODE tracks angles itself. Cited by include/ode/objects.h:2477.
- In `dAMotorEuler` mode the axes must be set correctly; the angle information is needed for stops but not for plain axis motors. Cited by include/ode/objects.h:3114, include/ode/objects.h:2478.
- Like any joint, the AMotor is inert until `dJointAttach` is called. Cited by include/ode/objects.h:1688.
- **`dParamVel` sign is body-attach-order-dependent — calibrate it, do not assume.** The relative angular
  velocity an AMotor commands follows ODE's internal body-pair ordering, which can be *opposite* to your own
  kinematic convention (e.g. a geometric Jacobian's `axis × r`). **Empirically established** by controlled
  gravity-off probes (not stated in the public headers): a **world-attached** motor `dJointAttach(am, 0, b)`
  is **consistent (+1)**, while an **interior two-dynamic-body** motor `dJointAttach(am, b0, b1)` is
  **flipped (−1)** for the same commanded `dParamVel` (`assets/probe_sign.cpp` → +1; `assets/probe_sign2.cpp`
  → −1). So store a **per-joint sign multiplier**: drive a single one-axis AMotor with a known `dParamVel`,
  read back the observable (tip motion / `dJointGetHingeAngleRate`), and bake in the measured ±1. Used in
  `assets/arm6dof.cpp:79-85,320-322`. See `references/foundations/verifying-simulations.md` §3 (falsification).

## Never invent

- `dJointSetMotorVelocity` / `dJointSetAMotorVel` — use `dJointSetAMotorParam(j, dParamVel, ...)`. Cited by `03-joints.json` never_invent.
- `dParamMotorVel` / `dParamMaxForce` — correct names are `dParamVel` / `dParamFMax`. Cited by `03-joints.json` never_invent.
- `dJointAddAMotorTorque` (singular) — the function is `dJointAddAMotorTorques`. Cited by include/ode/objects.h:2505.

Cross-link: references/joints.md (joints overview).

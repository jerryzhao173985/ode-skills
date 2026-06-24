# Foundations: cross-cutting recipes (HOWTO patterns)

> Practical "how do I…" patterns that span multiple components, synthesized from the ODE **FAQ**, the **manual**, the **demos**, and the issue tracker. These are *guidance* (cite the FAQ/manual/demo); the exact API lives in the linked `references/` files, which win on any conflict.

## Static / immovable objects
Make scenery (ground, walls) a **geom with NO body** — create the geom, never call `dBodyCreate`/`dGeomSetBody` for it; it returns body `0` in callbacks and `dJointAttach(j, body, 0)` treats `0` as the fixed world. **Do not** fake static with a huge-mass body whose position you reset each step. _(FAQ; `references/geoms-and-spaces.md`)_

## Connect a body to the world
`dJointAttach(joint, body, 0)` (or `(0, body)`) anchors a joint to the static environment. _(FAQ; `references/joints.md`)_

## A vehicle (chassis + wheels)
Chassis box body + one wheel body per wheel, each joined by `dJointCreateHinge2`: axis 1 = steering (z), axis 2 = spin (y). Drive with `dParamVel2`+`dParamFMax2`, steer proportionally with `dParamVel`+`LoStop/HiStop`, suspension via `dParamSuspensionERP/CFM`. Lock non-steering wheels with `LoStop=HiStop=0`. Put the car in its own sub-space + filter self-collision. _(Worked example: `references/examples/buggy.md`)_

## A servo motor (drive to a target angle)
Each step: `error = target − dJointGetHingeAngle(j)`; `dJointSetHingeParam(j, dParamVel, -error*gain)`; `dJointSetHingeParam(j, dParamFMax, maxForce)`. A velocity motor with a force budget *is* a position servo. _(FAQ; buggy steering)_

## Fast-spinning wheels look "bent"
Enable `dBodySetFiniteRotationMode` and update `dBodySetFiniteRotationAxis` each step to match the hinge axis — otherwise large per-step rotations accumulate numerical error. _(FAQ)_

## Conveyor belt / one-way moving surface
Attach a **contact joint to the world** (body `0`) and set the contact's surface **motion** fields (`dContactMotion1/2/N`) to the desired surface velocity. _(FAQ; `demo_motion.cpp`; `references/foundations/erp-cfm-friction.md`)_

## A composite (multi-geom) body
Build each sub-shape's `dMass`, `dMassAdd` them, `dMassTranslate(&m, -m.c…)` so the combined CoM sits at the body origin, then offset each geom from the body with `dGeomSetOffsetPosition`/`Rotation`. _(Worked example: `references/examples/boxstack.md`; `references/bodies-and-mass.md`)_

## A ragdoll / articulated character
A chain/tree of body segments joined by ball/hinge/universal joints with `dParamLoStop`/`dParamHiStop` limits to model anatomical ranges; position bodies first (anchors are global), then attach and set limits. _(Manual: Joints; `references/joints.md`)_

## Raycast / picking
Create a `dCreateRay(space, length)`, point it each frame with `dGeomRaySet(ray, px,py,pz, dx,dy,dz)`, and `dCollide` it (or run it through `dSpaceCollide`); use `dGeomRaySetClosestHit` for the nearest trimesh hit. _(`references/trimesh-heightfield.md`)_

## Make a simulation stable (checklist)
1. **Fixed, small step size** — never variable (`references/foundations/stepping-and-stability.md`).
2. **ERP ≈ 0.2**, small positive **CFM** (the defaults); raise CFM if it misbehaves near singularities (`references/tokens.md`).
3. **Soft resting contacts**: `dContactSoftCFM`/`dContactSoftERP` for stacks (`references/foundations/stepping-and-stability.md`).
4. **`dContactApprox1`** so friction scales with normal force (otherwise rolling bodies lock) _(FAQ)_.
5. **Auto-disable + `dWorldQuickStep`** for large stacks.
6. **Scale** so lengths/masses are ~`0.1–1.0` and masses within a few orders of magnitude (single-precision factorizer accuracy) _(FAQ)_.
7. Small **contact surface layer** (`~0.001`) and finite **max correcting velocity** to tame jitter/popping.

## Joint stops won't move?
An inconsistent stop where `hi < lo` is silently ignored — set **hi, then lo, then hi** again. _(FAQ)_

## Units
ODE is unit-agnostic but use a **consistent MKS/SI** system (meters, kilograms, seconds) to avoid ugly multipliers. _(FAQ)_

## Debugging numerical blow-ups
Enable FP exceptions (`feenableexcept(FE_DIVBYZERO|FE_INVALID)`) and run under UBSan/Valgrind to surface libccd div-by-zero / NaN early; expect a benign LCP "uninitialised value" report. _(Issue tracker; `references/foundations/known-issues.md`)_

## Direct3D + double precision
D3D switches the FPU to single precision — pass `D3DCREATE_FPU_PRESERVE` to `CreateDevice` so a double-precision ODE build keeps its precision. _(FAQ)_

## Closed kinematic loop (four-bar linkage, parallel gripper, small Stewart loop)
A loop of links closed by joints is **over-constrained** — the closing joint adds a redundant constraint the solver must reconcile. Three things an open chain never needs: (1) **soften the closing joint** with a small per-joint CFM — `dJointSetHingeParam(jClose, dParamCFM, 1e-6)` (`common.h`) — or the redundant constraint fights itself and jitters; (2) use **`dWorldStep`** (accurate LCP), not `dWorldQuickStep`, so the loop's coupled constraints converge; (3) drive the input with a motored hinge (`dParamVel`+`dParamFMax`) whose force ceiling can overpower the closing reaction. **Verify with the reaction force, not anchor separation** — for an over-constrained loop ERP heals the position so `|anchor−anchor2|` saturates; set `dJointSetFeedback` on the closing joint and assert its `dJointFeedback` force stays bounded — `assets/verify_harness.hpp`'s `JointForceProbe` wraps this exactly (`attach()` → `update()` each step → check `peakF`/`peakT`) (`references/foundations/verifying-simulations.md`). _(Field-proven: a four-bar where anchor-separation read ~3e-5 m whether intact or broken, but the closing force read 25.7 N intact vs 190-640 N broken.)_

## See also
`references/examples/` (runnable versions of these), `references/foundations/stepping-and-stability.md`, `references/foundations/{mental-model,erp-cfm-friction,known-issues}.md`.

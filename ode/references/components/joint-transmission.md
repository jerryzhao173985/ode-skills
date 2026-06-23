# Joint: Transmission (`dJointCreateTransmission`)

> Source of truth: include/ode/objects.h:1816. Cited by file:line; do not invent symbols.

## What it is

A transmission joint couples two bodies as a gear-like pair: each body acts as a wheel with its own axis, anchor and radius, and their angular velocities are coupled through a gear ratio. It supports three modes — `dTransmissionParallelAxes`, `dTransmissionIntersectingAxes`, `dTransmissionChainDrive`. Cited by include/ode/objects.h:1811, include/ode/objects.h:3244, include/ode/common.h:508.

## Create & attach

```c
dJointID j = dJointCreateTransmission(world, 0);     /* group 0 = normal alloc */
dJointAttach(j, body1, body2);                        /* required; both bodies are wheels */
dJointSetTransmissionMode(j, dTransmissionParallelAxes);
dJointSetTransmissionAxis(j, 0, 0, 1);                /* common axis for both wheels */
dJointSetTransmissionAnchor1(j, x1, y1, z1);          /* wheel-1 anchor, world coords */
dJointSetTransmissionAnchor2(j, x2, y2, z2);          /* wheel-2 anchor, world coords */
dJointSetTransmissionRatio(j, 2.0);                   /* gear ratio */
```

Cited by include/ode/objects.h:1816, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:3250 (mode), include/ode/objects.h:3285 (common axis), include/ode/objects.h:3208 (anchor1), include/ode/objects.h:3222 (anchor2), include/ode/objects.h:3267 (ratio).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetTransmissionAxis` | `void dJointSetTransmissionAxis (dJointID j, dReal x, dReal y, dReal z)` | include/ode/objects.h:3285 | Set one common axis of revolution for both wheels. |
| `dJointGetTransmissionAxis` | `void dJointGetTransmissionAxis (dJointID j, dVector3 result)` | include/ode/objects.h:3291 | Get the common axis into `result`. |
| `dJointSetTransmissionAxis1` | `void dJointSetTransmissionAxis1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3168 | Set the first wheel's axis (use when axes differ). |
| `dJointGetTransmissionAxis1` | `void dJointGetTransmissionAxis1 (dJointID, dVector3 result)` | include/ode/objects.h:3178 | Get the first wheel's axis. |
| `dJointSetTransmissionAxis2` | `void dJointSetTransmissionAxis2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3190 | Set the second wheel's axis. |
| `dJointGetTransmissionAxis2` | `void dJointGetTransmissionAxis2 (dJointID, dVector3 result)` | include/ode/objects.h:3200 | Get the second wheel's axis. |
| `dJointSetTransmissionAnchor1` | `void dJointSetTransmissionAnchor1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3208 | Set the first anchor (world coords). |
| `dJointGetTransmissionAnchor1` | `void dJointGetTransmissionAnchor1 (dJointID, dVector3 result)` | include/ode/objects.h:3214 | Get the first anchor. |
| `dJointSetTransmissionAnchor2` | `void dJointSetTransmissionAnchor2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3222 | Set the second anchor. |
| `dJointGetTransmissionAnchor2` | `void dJointGetTransmissionAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:3228 | Get the second anchor. |
| `dJointGetTransmissionContactPoint1` | `void dJointGetTransmissionContactPoint1 (dJointID, dVector3 result)` | include/ode/objects.h:3151 | Get the first wheel's contact point. |
| `dJointGetTransmissionContactPoint2` | `void dJointGetTransmissionContactPoint2 (dJointID, dVector3 result)` | include/ode/objects.h:3157 | Get the second wheel's contact point. |
| `dJointSetTransmissionParam` | `void dJointSetTransmissionParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:3234 | Set a transmission `dParam*` parameter. |
| `dJointGetTransmissionParam` | `dReal dJointGetTransmissionParam (dJointID, int parameter)` | include/ode/objects.h:3240 | Read back a transmission `dParam*` value. |
| `dJointSetTransmissionMode` | `void dJointSetTransmissionMode (dJointID j, int mode)` | include/ode/objects.h:3250 | Set the transmission mode. |
| `dJointGetTransmissionMode` | `int dJointGetTransmissionMode (dJointID j)` | include/ode/objects.h:3256 | Get the transmission mode. |
| `dJointSetTransmissionRatio` | `void dJointSetTransmissionRatio (dJointID j, dReal ratio)` | include/ode/objects.h:3267 | Set the gear ratio. |
| `dJointGetTransmissionRatio` | `dReal dJointGetTransmissionRatio (dJointID j)` | include/ode/objects.h:3273 | Get the gear ratio. |
| `dJointGetTransmissionAngle1` | `dReal dJointGetTransmissionAngle1 (dJointID j)` | include/ode/objects.h:3298 | Get the first wheel's angle. |
| `dJointGetTransmissionAngle2` | `dReal dJointGetTransmissionAngle2 (dJointID j)` | include/ode/objects.h:3305 | Get the second wheel's angle. |
| `dJointGetTransmissionRadius1` | `dReal dJointGetTransmissionRadius1 (dJointID j)` | include/ode/objects.h:3311 | Get the first wheel's radius. |
| `dJointGetTransmissionRadius2` | `dReal dJointGetTransmissionRadius2 (dJointID j)` | include/ode/objects.h:3317 | Get the second wheel's radius. |
| `dJointSetTransmissionRadius1` | `void dJointSetTransmissionRadius1 (dJointID j, dReal radius)` | include/ode/objects.h:3326 | Set the first wheel's radius. |
| `dJointSetTransmissionRadius2` | `void dJointSetTransmissionRadius2 (dJointID j, dReal radius)` | include/ode/objects.h:3335 | Set the second wheel's radius. |
| `dJointGetTransmissionBacklash` | `dReal dJointGetTransmissionBacklash (dJointID j)` | include/ode/objects.h:3341 | Get the backlash. |
| `dJointSetTransmissionBacklash` | `void dJointSetTransmissionBacklash (dJointID j, dReal backlash)` | include/ode/objects.h:3361 | Set the backlash. |

## Parameters / conventions

- Mode is one of `dTransmissionParallelAxes` (0), `dTransmissionIntersectingAxes` (1), `dTransmissionChainDrive` (2). Cited by include/ode/objects.h:3244, include/ode/common.h:508.
- Use `dJointSetTransmissionAxis` for a single common axis of revolution for both gears; use `dJointSetTransmissionAxis1` / `dJointSetTransmissionAxis2` when the wheels have different axes. The common axis is given in world coordinates. Cited by include/ode/objects.h:3276, include/ode/objects.h:3160, include/ode/objects.h:3282.
- Each wheel has its own anchor (`Anchor1`/`Anchor2`), radius (`Radius1`/`Radius2`), angle getter, and contact point; coupling strength is the gear `Ratio`. Cited by include/ode/objects.h:3203, include/ode/objects.h:3320, include/ode/objects.h:3259.

## Pitfalls

- Set axes/anchors/ratio *after* attaching; an unattached joint has no effect. Cited by include/ode/objects.h:1688.
- Mixing the single common-axis setter with the per-wheel `Axis1`/`Axis2` setters is redundant — `dJointSetTransmissionAxis` sets the axis of revolution for *both* gears at once. Cited by include/ode/objects.h:3278.
- Radius/angle/contact-point values are per-wheel; do not assume a single shared value. Cited by include/ode/objects.h:3308.
- (Observed; header-silent) `dJointGetTransmissionRadius1`/`Radius2` (and the other transmission read-backs) return 0 *before* the first `dWorldStep` — the solver lazily computes the wheel geometry on the first step, so reading radii right after `dJointSetTransmission*` "proves" wrong (zero) radii. Step once, then read. The header declares only `dReal dJointGetTransmissionRadius1(dJointID j)`. Cited by include/ode/objects.h:3311, include/ode/objects.h:3317.
- (Observed; header-silent) If the two meshing gear bodies also carry geoms that collide, the cylinder-cylinder *contact* injects spurious torque that fights the transmission coupling (you see jitter/drift the joint never asked for). Make `dJointCreateTransmission` the SOLE coupling: suppress that geom pair in the collision callback (skip the contact, or use a category/collide-bits mask) or give the gear bodies no geoms at all. Cited by include/ode/objects.h:1816.

## Never invent

- `dJointCreateGearbox` — the function is `dJointCreateTransmission`. (grep `include/ode/objects.h` — not a real symbol)
- `dJointSetTransmissionGearRatio` — the setter is `dJointSetTransmissionRatio`. Cited by include/ode/objects.h:3267.
- `dTransmissionGear` — modes are `dTransmissionParallelAxes` / `dTransmissionIntersectingAxes` / `dTransmissionChainDrive`. Cited by include/ode/common.h:508.

Cross-link: references/joints.md (joints overview).

# Geom: Cylinder  (`dCreateCylinder`)

> Source of truth: include/ode/collision.h:1062. Cited by file:line; do not invent symbols.

## What it is

A flat-ended cylinder geom defined by a **radius** and a **length**. Unlike a capsule, the ends are flat caps, so `length` is the **full** end-to-end height of the cylinder. The long axis is local **Z**, centered at the reference point (`include/ode/collision.h:1062`).

## Create & attach

```c
dGeomID cyl = dCreateCylinder (space, radius, length);  /* flat ends; length = full height */
dGeomSetBody (cyl, body);                                 /* bind to a body; 0 detaches */
```

Binding to a body makes the geom share the body's pose; `body = 0` detaches and gives the geom its own pose (`include/ode/collision.h:105`).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomCylinderSetParams` | `void dGeomCylinderSetParams (dGeomID cylinder, dReal radius, dReal length)` | include/ode/collision.h:1063 | Set radius and full length. |
| `dGeomCylinderGetParams` | `void dGeomCylinderGetParams (dGeomID cylinder, dReal *radius, dReal *length)` | include/ode/collision.h:1064 | Read radius and length into the two out-pointers. |

## Parameters / conventions

- **Dimension semantics**: `radius` + `length`, where `length` is the **full** flat-ended height (no caps to subtract, in contrast to the capsule) (`include/ode/collision.h:1062`).
- **Get-dims function**: `dGeomCylinderGetParams` writes through two `dReal*` out-pointers — pass `&radius, &length` (`include/ode/collision.h:1064`).
- **Axis**: the cylinder extends along its local **Z** axis; reorient via the geom/body rotation or `dGeomSetOffsetRotation` (`include/ode/collision.h:558`).
- **Class**: `dGeomGetClass` returns `dCylinderClass` (`include/ode/collision.h:307`, class ids at `:877`).

## Pitfalls

- **Cylinder length is full, capsule length is not**: a cylinder of `length L` is `L` tall, whereas a capsule's `length` excludes its two caps. Do not copy capsule sizing math onto a cylinder (`include/ode/collision.h:1062`).
- `dGeomCylinderGetParams` returns `void` and fills both out-pointers; there is no scalar-returning getter (`include/ode/collision.h:1064`).
- Cylinder-vs-cylinder/box collision is historically the weakest collider in ODE; for tall thin rollers a capsule is often more robust — but that is a stability note, not an API rule. [VERIFY]

## Never invent

- `dGeomCylinderGetRadius` / `dGeomCylinderGetLength` — both dimensions come together via `dGeomCylinderGetParams`.
- `dCreateCCylinder` for a *flat*-ended cylinder — `dCreateCCylinder` aliases `dCreateCapsule` (round-ended), NOT `dCreateCylinder` (`include/ode/collision.h:1056`).
- `dCylinderCreate` — the constructor is `dCreateCylinder`.

---

See the overview: [geoms-and-spaces.md](../geoms-and-spaces.md).

# Geom: Capsule  (`dCreateCapsule`)

> Source of truth: include/ode/collision.h:1050. Cited by file:line; do not invent symbols.

## What it is

A capsule (round-ended cylinder) geom defined by a **radius** and a **length**, where `length` is the length of the central **cylinder only — it EXCLUDES the two hemispherical caps**. The total end-to-end span is therefore `length + 2*radius`. The capsule's long axis is local **Z**, centered at the reference point (`include/ode/collision.h:1050`).

## Create & attach

```c
dGeomID cap = dCreateCapsule (space, radius, length);  /* length EXCLUDES the 2 caps */
dGeomSetBody (cap, body);                                /* bind to a body; 0 detaches */
```

`dCreateCCylinder` is a backward-compatible `#define` alias for `dCreateCapsule` — it is **not** a separate function (`include/ode/collision.h:1056`).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomCapsuleSetParams` | `void dGeomCapsuleSetParams (dGeomID ccylinder, dReal radius, dReal length)` | include/ode/collision.h:1051 | Set radius and cylinder length (caps excluded). |
| `dGeomCapsuleGetParams` | `void dGeomCapsuleGetParams (dGeomID ccylinder, dReal *radius, dReal *length)` | include/ode/collision.h:1052 | Read radius and cylinder length into the two out-pointers. |
| `dGeomCapsulePointDepth` | `dReal dGeomCapsulePointDepth (dGeomID ccylinder, dReal x, dReal y, dReal z)` | include/ode/collision.h:1053 | Depth of a point inside the capsule (>0 inside). |

## Parameters / conventions

- **Dimension semantics**: `radius` + `length`, where `length` is the cylinder section ONLY and **excludes** the two hemispherical end caps; full length = `length + 2*radius` (`include/ode/collision.h:1050`).
- **Get-dims function**: `dGeomCapsuleGetParams` writes through two `dReal*` out-pointers — pass `&radius, &length` (`include/ode/collision.h:1052`).
- **Axis**: the capsule extends along its local **Z** axis; reorient by setting the geom/body rotation or `dGeomSetOffsetRotation` (`include/ode/collision.h:558`).
- **Mass call-site — `direction` argument**: `dMassSetCapsule(dMass*, density, int direction, radius, length)` and `dMassSetCapsuleTotal(dMass*, total_mass, int direction, radius, length)` take a `direction` arg before the dims (`include/ode/mass.h:55`, `:57`). The header declares `int direction` but does not document its values; observed in field builds, `direction` selects the local long axis as `1=x, 2=y, 3=z`. Since the collision geom's long axis is local **Z** (above), **pass `direction = 3`** so the inertia tensor aligns with the geom shape. The C++ wrappers `setCapsule` / `setCapsuleTotal` forward the same `direction` arg (`include/ode/mass.h:108`, `:110`).
- **Create call-site — `length` excludes the caps**: `dCreateCapsule(space, radius, length)` interprets `length` as the central cylinder only, NOT tip-to-tip; the total long-axis span is `length + 2*radius` (`include/ode/collision.h:1050`). Keep `length` consistent between the geom (`dCreateCapsule`) and the mass (`dMassSetCapsule`) — both use the cap-excluded cylinder length.
- **Aliases**: `dGeomCCylinderSetParams` / `dGeomCCylinderGetParams` / `dGeomCCylinderPointDepth` are `#define` aliases of the `Capsule` calls (`include/ode/collision.h:1057`).
- **Class**: `dGeomGetClass` returns `dCapsuleClass` (`dCCylinderClass` is a `#define` alias) (`include/ode/collision.h:307`, class ids at `:877`).

## Pitfalls

- **Length excludes the caps**: `dCreateCapsule(space, r, L)` produces a shape `L + 2*r` tall, not `L` tall (`include/ode/collision.h:1050`). When matching a desired total height `H`, pass `length = H - 2*radius`.
- `dGeomCapsuleGetParams` returns `void` and fills both out-pointers; there is no scalar-returning getter (`include/ode/collision.h:1052`).
- `dCreateCCylinder` is only a name alias — do not treat it as a distinct geom type with its own parameters (`include/ode/collision.h:1056`).

## Never invent

- `dGeomCapsuleGetRadius` / `dGeomCapsuleGetLength` — both dimensions come together via `dGeomCapsuleGetParams`.
- `dCreateCapsule(space, radius, totalLength)` interpreted as full length — `length` is the cap-excluded cylinder length.
- `dCapsuleCreate` — the constructor is `dCreateCapsule` (alias `dCreateCCylinder`).

---

See the overview: [geoms-and-spaces.md](../geoms-and-spaces.md).

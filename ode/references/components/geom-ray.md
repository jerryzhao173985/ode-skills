# Geom: Ray  (`dCreateRay`)

> Source of truth: include/ode/collision.h:1066. Cited by file:line; do not invent symbols.

## What it is

A ray geom is a **half-line of finite length** with a start point and a direction; collide it against other geoms (with `dCollide`) to find the closest hit — used for line-of-sight, wheel ground probes, picking, and laser-style queries. Its geometry is a length plus a start point and direction set via `dGeomRaySet` (`include/ode/collision.h:1066`).

## Create & attach

A ray is placeable, but you normally set its start/direction directly with `dGeomRaySet` rather than via a body.

```c
dGeomID ray = dCreateRay (space, 5.0);             /* length = 5.0 */
dGeomRaySet (ray, 0,0,1,  0,0,-1);                 /* start (0,0,1), direction down (0,0,-1) */
/* Optionally bind to a body so it follows that body's pose: */
/* dGeomSetBody (ray, body); */
```

The direction passed to `dGeomRaySet` is normalized internally; the ray then extends `length` units from the start point along that direction (`include/ode/collision.h:1066`, `include/ode/collision.h:1069`).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateRay` | `dGeomID dCreateRay (dSpaceID space, dReal length)` | include/ode/collision.h:1066 | Create a ray geom of the given length. |
| `dGeomRaySetLength` | `void dGeomRaySetLength (dGeomID ray, dReal length)` | include/ode/collision.h:1067 | Change the ray's length. |
| `dGeomRayGetLength` | `dReal dGeomRayGetLength (dGeomID ray)` | include/ode/collision.h:1068 | Read the ray's length. |
| `dGeomRaySet` | `void dGeomRaySet (dGeomID ray, dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)` | include/ode/collision.h:1069 | Set start point `(px,py,pz)` and direction `(dx,dy,dz)`. |
| `dGeomRayGet` | `void dGeomRayGet (dGeomID ray, dVector3 start, dVector3 dir)` | include/ode/collision.h:1071 | Read back start point and direction. |
| `dGeomRaySetFirstContact` | `void dGeomRaySetFirstContact (dGeomID g, int firstContact)` | include/ode/collision.h:1080 | Choose first-contact vs. closest-hit behavior. |
| `dGeomRaySetBackfaceCull` | `void dGeomRaySetBackfaceCull (dGeomID g, int backfaceCull)` | include/ode/collision.h:1082 | Toggle backface culling for trimesh hits. |
| `dGeomRaySetClosestHit` | `void dGeomRaySetClosestHit (dGeomID g, int closestHit)` | include/ode/collision.h:1084 | Return the closest hit instead of any hit. |

## Parameters / conventions

- **Length** is a scalar (`dCreateRay`/`dGeomRaySetLength`); start + direction are set together with `dGeomRaySet` and read with `dGeomRayGet` — the direction component is normalized by ODE.
- **Hit-selection flags** (`dGeomRaySetFirstContact`, `dGeomRaySetBackfaceCull`, `dGeomRaySetClosestHit`) tune which of several candidate intersections `dCollide` reports; each has a matching getter (`dGeomRayGetFirstContact` etc., `include/ode/collision.h:1081`,`:1083`,`:1085`).
- A ray is placeable, so `dGeomSetBody`/offsets apply, but `dGeomRaySet` overrides the start/direction directly in world space.

## Pitfalls

- A ray reports contacts but exerts **no** physical force by itself — it is a query primitive; it only generates contacts when you call `dCollide` against it.
- `dGeomRaySet` takes **six** scalars (start xyz + direction xyz), not a position only; the direction is the second triple `(dx,dy,dz)` (`include/ode/collision.h:1069`).
- The default hit mode may return *a* hit rather than the *closest*; call `dGeomRaySetClosestHit(ray, 1)` when you need the nearest intersection (`include/ode/collision.h:1084`).
- `dGeomRayGetParams` exists but is `ODE_API_DEPRECATED`; prefer the individual `dGeomRaySet*`/`dGeomRayGet*` flag accessors (`include/ode/collision.h:1079`). [VERIFY] the deprecation marker on `dGeomRayGetParams` (line 1079, not in the inventory JSON).

## Never invent

- `dGeomRaySetDirection` / `dGeomRaySetStart` — start and direction are set together via `dGeomRaySet`.
- `dCreateLine` / `dCreateSegment` — the only line primitive is `dCreateRay`.

See also the geoms overview: `references/geoms-and-spaces.md`.

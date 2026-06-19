# Geom: Sphere  (`dCreateSphere`)

> Source of truth: include/ode/collision.h:921. Cited by file:line; do not invent symbols.

## What it is

A sphere geom defined by a single **radius**. It is a placeable geom whose reference point is its **center** (`include/ode/collision.h:921`).

## Create & attach

```c
dGeomID sphere = dCreateSphere (space, radius);   /* space may be 0 */
dGeomSetBody (sphere, body);                       /* bind to a body; 0 detaches */
```

Binding to a body makes the geom share the body's position/rotation (`include/ode/collision.h:105`). Verbatim-style from the demo, where each sphere geom is attached to its body (`ode/demo/demo_buggy.cpp:249`):

```c
dGeomSetBody (sphere[i-1], body[i]);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomSphereSetRadius` | `void dGeomSphereSetRadius (dGeomID sphere, dReal radius)` | include/ode/collision.h:933 | Change the sphere's radius. |
| `dGeomSphereGetRadius` | `dReal dGeomSphereGetRadius (dGeomID sphere)` | include/ode/collision.h:944 | Read the sphere's radius. |
| `dGeomSpherePointDepth` | `dReal dGeomSpherePointDepth (dGeomID sphere, dReal x, dReal y, dReal z)` | include/ode/collision.h:961 | Depth of a point inside the sphere (>0 inside). |

## Parameters / conventions

- **Dimension**: one scalar `radius`. There is no separate get-dimensions function returning multiple values — the radius is returned directly by `dGeomSphereGetRadius` (`include/ode/collision.h:944`).
- **Reference point**: the sphere's center (`include/ode/collision.h:921`).
- **Offsets**: to displace the geom from its body's center of mass use `dGeomSetOffsetPosition` / `dGeomSetOffsetRotation` (`include/ode/collision.h:542`, `:558`), which require the geom to be attached to a body.
- **Class**: `dGeomGetClass` returns `dSphereClass` (`include/ode/collision.h:307`, class ids at `:877`).

## Pitfalls

- A geom is not a body: `dCreateSphere` only creates collision geometry. To make it dynamic you must create a body, bind it with `dGeomSetBody`, and give the body a sphere mass separately (`include/ode/collision.h:105`).
- `dGeomSetPosition`/`dGeomSetRotation` only work because a sphere is placeable; the same calls on a non-placeable geom (plane/fixed heightfield) are a debug-build runtime error (`include/ode/collision.h:131`).
- Candidate pairs from `dSpaceCollide` are not guaranteed to intersect — always check `dCollide` returns `n>0` before building contact joints (`include/ode/collision.h:812`).

## Never invent

- `dGeomSphereGetParams` / `dGeomSphereSetParams` — spheres use `dGeomSphereGetRadius`/`dGeomSphereSetRadius`, not a `Params` pair.
- `dGeomSetRadius` (unqualified) — the function is `dGeomSphereSetRadius`.
- `dSphereCreate` — the constructor is `dCreateSphere`.

---

See the overview: [geoms-and-spaces.md](../geoms-and-spaces.md).

# Geom: Convex  (`dCreateConvex`)

> Source of truth: include/ode/collision.h:965. Cited by file:line; do not invent symbols.

## What it is

A convex-hull geom built from three caller-owned arrays: **face plane equations**, a **point cloud** (the hull vertices), and a **polygon index list** describing each face. It collides as the convex solid enclosed by those planes (`include/ode/collision.h:965`).

## Create & attach

The three data arrays are **referenced, not copied** — they must outlive the geom (`.extract-ds-skill-scratch/deep-inv/trimesh-heightfield.json`, cite include/ode/collision.h:965).

```c
/* planes : 4 dReals per face (a,b,c,d)
 * points : 3 dReals per vertex (x,y,z)
 * polygons: flat list, per face: <vertexCount> then that many point indices */
dGeomID geom = dCreateConvex (space,
                              planes,   planecount,
                              points,   pointcount,
                              polygons);
dGeomSetBody (geom, body);            /* convex is placeable; bind to a body */
```

Verbatim shape from `ode/demo/demo_convex.cpp:257`:

```c
dGeomID geom = dCreateConvex
(
    space,
    halton_planes[i],
    halton_numf[i],
    halton_verts[i],
    halton_numv[i],
    halton_faces[i]
);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateConvex` | `dGeomID dCreateConvex (dSpaceID space, const dReal *_planes, unsigned int _planecount, const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons)` | include/ode/collision.h:965 | Create a convex geom from planes, points, and polygon index data. |
| `dGeomSetConvex` | `void dGeomSetConvex (dGeomID g, const dReal *_planes, unsigned int _count, const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons)` | include/ode/collision.h:972 | Re-set the geometry data of an existing convex geom (same layout). |

## Parameters / conventions

- **`_planes`**: 4 `dReal`s per face — a plane equation `a,b,c,d`. **`_planecount`** is the number of faces.
- **`_points`**: 3 `dReal`s per vertex `(x,y,z)`. **`_pointcount`** is the number of vertices.
- **`_polygons`**: a flat `unsigned int` list; for each face a count followed by that many point indices (`<vertexCount, idx0, idx1, ...>` per face) (`.extract-ds-skill-scratch/deep-inv/trimesh-heightfield.json`, survey of include/ode/collision.h:964-978).
- Convex is **placeable**: bind to a body with `dGeomSetBody`, or offset from a body with `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation` (as in `ode/demo/demo_convex.cpp:257`).

## Pitfalls

- The `_planes`/`_points`/`_polygons` arrays are referenced, not copied — they (and any per-instance buffer you build them in) must outlive the geom or collision reads freed memory.
- [VERIFY] the exact winding/normal convention required by `_planes`/`_polygons` is not fully specified in the header (only the signature is at include/ode/collision.h:965-978); the full contract lives in the convex backend / `demo_convex.cpp` and was not read here.
- `_planecount` (`dCreateConvex`) and `_count` (`dGeomSetConvex`) name the same face-count argument — match it to the number of plane equations, not the number of points.

## Never invent

- `dGeomConvexGetPlanes` / `dGeomConvexGetPoints` — no such accessors; keep your own pointers to the arrays you passed in.
- `dCreateConvexHull` — the function is `dCreateConvex` (you supply a precomputed hull; ODE does not compute one for you here).

See also the geoms overview: `references/geoms-and-spaces.md`.

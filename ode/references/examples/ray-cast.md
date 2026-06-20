# Example: ray casting — picking / line-of-sight with a ray geom

> Source: wiki **Simple ray casting query** (`ode.org/wiki/index.php/Simple_ray_casting_query`), grounded against `include/ode/collision.h`. ODE has **no built-in raycast** — you compose one from a ray geom + `dSpaceCollide2`.

## What it builds
A closest-hit query: cast a finite ray into a space and find the nearest geom it strikes (picking, ground sensing, line-of-sight).

## Components composed
A **ray Geom** (`dCreateRay`, created with space `0` so it isn't auto-collided) tested against a **Space** via `dSpaceCollide2`; the near-callback runs `dCollide` and keeps the minimum-distance hit. No bodies/joints needed.

## The pattern
```c
dGeomID Ray = dCreateRay(0, Length);                 // collision.h:1066 — space 0: not in any space
dGeomRaySet(Ray, sx,sy,sz, dx,dy,dz);                // collision.h:1069 — origin + (normalized) dir

dVector4 HitPosition;
HitPosition[3] = dInfinity;                          // [3] = running best distance
dSpaceCollide2(Ray, (dGeomID)Space, HitPosition, &RayCallback);   // collision.h:865
// a hit exists iff HitPosition[3] != dInfinity
dGeomDestroy(Ray);                                   // collision.h:65
```
```c
#define MAX_CONTACTS 32
void RayCallback(void *data, dGeomID g1, dGeomID g2) {
    dContact c[MAX_CONTACTS];
    int n = dCollide(g1, g2, MAX_CONTACTS, &c[0].geom, sizeof(dContact));  // collision.h:792
    for (int i = 0; i < n; i++)
        if (c[i].geom.depth < ((dReal*)data)[3]) {   // for a RAY, depth == distance along the ray
            dCopyVector3((dReal*)data, c[i].geom.pos);
            ((dReal*)data)[3] = c[i].geom.depth;
        }
}
```

## The key reinterpretation
For a ray collision, `dContactGeom` (`contact.h:88`) means: `pos` = world hit point, **`depth` = distance along the ray** (not penetration), `normal` = surface normal, `g1/g2` = the geoms (one is the ray).

## Two modes
- Against **one** geom: `dCollide(ray, other, N, …)` directly.
- Against a **whole space**: `dSpaceCollide2(ray, (dGeomID)space, data, cb)` then `dCollide` per pair in the callback.

## Ray tuning (full API in `references/trimesh-heightfield.md`)
`dGeomRaySetLength`/`dGeomRayGetLength` (`collision.h:1067-1068`), `dGeomRaySetClosestHit` (`collision.h:1084`, nearest trimesh hit), `dGeomRaySetFirstContact`/`SetBackfaceCull` — these chiefly affect ray-vs-**trimesh**.

## Gotchas
- Create the ray with space **`0`** so `dSpaceCollide` doesn't collide it automatically — you drive it via `dSpaceCollide2`.
- **Normalize the direction** before `dGeomRaySet` (the wiki normalizes by hand with `dCalcVectorLength3`/`dScaleVector3` — see `references/math-and-rotation.md`).
- For ray-vs-space the portable closest-hit is the manual min-depth loop above; the `ClosestHit` flag only changes trimesh behavior.
- **`dSpaceCollide2(ray, space, …)` against a whole space is an O(n) LINEAR SCAN, not broadphase-accelerated** (implementation property, not an API promise): the hash space (`ode/src/collision_space.cpp:622` — "could … avoid O(n2) … but it does not yet") and the SAP space (`ode/src/collision_sapspace.cpp:502` — "just a simple N^2 implementation") both AABB-test every member. So a single "is X hitting anything?" query is O(n) — `dSpaceCollide(space,…)` is accelerated only for the space's *own* internal pairs, not an outside geom. To scale per-frame ray/proximity queries, **nest spaces** so each query sweeps a small sub-space (the quadtree space is the exception — `dxQuadTreeSpace::collide2` tests a member only against its block + parents, `ode/src/collision_quadtreespace.cpp:586`), or accept the O(n).

## See also
`references/trimesh-heightfield.md` (`dCreateRay` family), `references/geoms-and-spaces.md`, `references/collision-and-contacts.md` (`dCollide`/`dSpaceCollide2`).

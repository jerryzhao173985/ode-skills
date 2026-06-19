# Example: heightfield — procedural terrain from a height callback

> Source: `ode/demo/demo_heightfield.cpp` (714 lines). Verified line anchors. The demo for **terrain (`dCreateHeightfield`)** and the callback build path. Pairs with `references/trimesh-heightfield.md`.

## What it builds
A terrain geom whose heights come from a user callback `dReal f(void*, int x, int z)`; dropped bodies (box/sphere/capsule/cylinder/convex/composite/trimesh) collide against it. The same callback is re-used to render the terrain as triangles.

## Components composed
- **World** + **Space** + contact **JointGroup**.
- A **heightfield Geom** built via the data-object pipeline (see below).
- A backstop **Geom** `dCreatePlane` beneath the finite terrain to catch objects rolling off the edge.
- Dropped **Body**+**Geom** objects colliding with the terrain.

## Key calls (walkthrough)
| Line | Call | Role |
|---|---|---|
| 156 | `heightfield_callback` | `dReal(void*,int x,int z)` — sampled per grid cell |
| 665 | `dGeomHeightfieldDataCreate` | allocate the data object |
| 668 | `dGeomHeightfieldDataBuildCallback(...)` | bind callback + dims + scale/offset/thickness/wrap |
| 674 | `dGeomHeightfieldDataSetBounds` | finite AABB (performance — defaults to ±∞) |
| 676 | `dCreateHeightfield(space, data, 1)` | the placeable geom |
| 686/689 | `dRFromAxisAndAngle` + `dGeomSetRotation` | rotate 90° about X → native +Y-up becomes Z-up |
| 660 | `dCreatePlane` | edge backstop |
| 582/585 | `dSpaceCollide` → `dWorldQuickStep` | the loop |
| 711 | `dGeomHeightfieldDataDestroy` | **app owns** the data — free it after geom/space teardown |

## Patterns to copy
- **Build pipeline:** `…DataCreate` → `…BuildCallback`/`BuildByte/Short/Single` → `…DataSetBounds` → `dCreateHeightfield`. Keep the data object alive for the geom's lifetime; destroy it yourself.
- **Reorient:** ODE authors a heightfield in X-Z with height along +Y — rotate 90° about X for a Z-up world.
- **Render terrain yourself:** drawstuff has no heightfield primitive; re-sample the same callback and emit `dsDrawTriangle` per cell.

## Gotchas
- The `dHeightfieldDataID` is **application-owned** — `dGeomHeightfieldDataDestroy` after the geom/space are gone, and it must outlive the geom.
- `…SetBounds` matters for performance (±∞ AABB → useless broadphase culling).
- A finite field has edges — add a plane backstop and spawn objects within the extent.
- **Tunneling:** small objects fall through ridges if the cell size isn't smaller than the object ([known-issues #71]).

## See also
`references/trimesh-heightfield.md` (heightfield API), `references/geoms-and-spaces.md`, `references/foundations/known-issues.md`.

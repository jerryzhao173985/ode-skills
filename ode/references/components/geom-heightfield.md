# Geom: Heightfield  (`dCreateHeightfield`)

> Source of truth: include/ode/collision.h:1147. Cited by file:line; do not invent symbols.

## What it is

A terrain geom: a regular grid of sample heights over a width × depth region. You build a separate **`dHeightfieldDataID`** (extents, sample grid, scale/offset/thickness/wrap, bounds), then create the geom from it. By default height runs along the geom's **local Y axis** with the field in the local x/z plane (`include/ode/collision.h:1147`).

## Create & attach

Heightfield uses a separate data lifecycle: create data → one `Build*` call → (optionally set bounds) → `dCreateHeightfield`. **The data object and any non-copied source buffer must outlive the geom** (`include/ode/collision.h:1147`).

```c
dReal hf_cb (void*, int x, int z) { /* return height at grid (x,z) */ }

dHeightfieldDataID data = dGeomHeightfieldDataCreate ();
dGeomHeightfieldDataBuildCallback (data, NULL, hf_cb,
        width, depth, widthSamples, depthSamples,
        1.0, 0.0, 0.0, 0);                 /* scale, offset, thickness, bWrap */
dGeomHeightfieldDataSetBounds (data, -4.0, 6.0);   /* BEFORE binding — AABB is fixed once */

dGeomID gheight = dCreateHeightfield (space, data, 1);   /* bPlaceable=1 => movable/rotatable */
/* Z-up world: height is along local Y, so rotate 90deg about X. */
/* On shutdown: dGeomDestroy(gheight); then dGeomHeightfieldDataDestroy(data); */
```

Verbatim shape from `ode/demo/demo_heightfield.cpp:665-690`:

```c
dGeomHeightfieldDataBuildCallback(heightid, NULL, heightfield_callback,
        HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
        REAL(1.0), REAL(0.0), REAL(0.0), 0);
dGeomHeightfieldDataSetBounds(heightid, REAL(-4.0), REAL(+6.0));
dGeomID gheight = dCreateHeightfield(space, heightid, 1);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateHeightfield` | `dGeomID dCreateHeightfield (dSpaceID space, dHeightfieldDataID data, int bPlaceable)` | include/ode/collision.h:1147 | Create the heightfield geom; `bPlaceable` non-zero allows `dGeomSetPosition`/`dGeomSetRotation`. |
| `dGeomHeightfieldDataCreate` | `dHeightfieldDataID dGeomHeightfieldDataCreate (void)` | include/ode/collision.h:1163 | Allocate an empty data object (you own it, not ODE). |
| `dGeomHeightfieldDataDestroy` | `void dGeomHeightfieldDataDestroy (dHeightfieldDataID d)` | include/ode/collision.h:1174 | Free the data object — only after the geom is destroyed. |
| `dGeomHeightfieldDataBuildCallback` | `void dGeomHeightfieldDataBuildCallback (dHeightfieldDataID d, void* pUserData, dHeightfieldGetHeight* pCallback, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1218 | Configure heights via a per-`(x,z)` callback. |
| `dGeomHeightfieldDataBuildByte` | `void dGeomHeightfieldDataBuildByte (dHeightfieldDataID d, const unsigned char* pHeightData, int bCopyHeightData, ...)` | include/ode/collision.h:1266 | Configure from an 8-bit unsigned height array. |
| `dGeomHeightfieldDataBuildShort` | `void dGeomHeightfieldDataBuildShort (dHeightfieldDataID d, const short* pHeightData, int bCopyHeightData, ...)` | include/ode/collision.h:1314 | Configure from a 16-bit signed short array. |
| `dGeomHeightfieldDataBuildSingle` | `void dGeomHeightfieldDataBuildSingle (dHeightfieldDataID d, const float* pHeightData, int bCopyHeightData, ...)` | include/ode/collision.h:1364 | Configure from a single-precision `float` array (`float*` regardless of `dReal`). |
| `dGeomHeightfieldDataBuildDouble` | `void dGeomHeightfieldDataBuildDouble (dHeightfieldDataID d, const double* pHeightData, int bCopyHeightData, ...)` | include/ode/collision.h:1414 | Configure from a double-precision `double` array (`double*` regardless of `dReal`). |
| `dGeomHeightfieldDataSetBounds` | `void dGeomHeightfieldDataSetBounds (dHeightfieldDataID d, dReal minHeight, dReal maxHeight)` | include/ode/collision.h:1436 | Set min/max height for the AABB; **must** precede binding. |
| `dGeomHeightfieldSetHeightfieldData` | `void dGeomHeightfieldSetHeightfieldData (dGeomID g, dHeightfieldDataID d)` | include/ode/collision.h:1450 | Rebind data to an existing geom without changing its placeable flag. |
| `dGeomHeightfieldGetHeightfieldData` | `dHeightfieldDataID dGeomHeightfieldGetHeightfieldData (dGeomID g)` | include/ode/collision.h:1462 | Read the data bound to a heightfield geom (NULL if none). |

The `Build*` array variants share the trailing args `dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap` (abbreviated `...` above; full signatures at the cited lines).

## Parameters / conventions

- **Orientation**: height is along the geom's **local Y axis**; the field lies in local x/z. For a Z-up world, create it placeable (`bPlaceable=1`) and rotate 90° about X (`include/ode/collision.h:1147`). The header states only that an unplaceable heightfield uses "the global y axis" for height (`include/ode/collision.h:1140`); the placeable rotation/axis-mapping below is observed from field builds, not stated in the header.
- **Z-up axis mapping + inverse (spawn placement)**: after `dRFromAxisAndAngle(R,1,0,0,M_PI/2)` (`include/ode/rotation.h:36`) the local→world axis mapping is:

  | local axis | maps to world | notes |
  | --- | --- | --- |
  | local X | world +X | grid width unchanged |
  | local Y (height) | world +Z | "up" — what you want |
  | local Z | world −Y | grid depth is **negated** |

  ```
   local +Y (height)              world +Z (height)
        |                              |
        |          +90° about X        |
        +----- local +X      ===>      +----- world +X
       /                              /
   local +Z                      world −Y   (Z -> −Y)
  ```

  **Inverse — to spawn a body ON the terrain at world `(X, Y)`**: the height callback / sample grid is indexed in *local* `(x, z)`, and local `z = −worldY`. So evaluate your height function at **`(X, −Y)`** and place the body at `world (X, Y, h)` where `h = heightFn(X, −Y)` (plus scale/offset per the build call). Forgetting the sign on the second coordinate (sampling `(X, Y)` instead of `(X, −Y)`) reads the wrong row of the field — the body spawns embedded or floating. (Mapping observed/empirical; the −90°-vs-+90° choice and the `z=−worldY` sign are *not* spelled out in the header — a wrong rotation here silently buries or inverts the terrain.)
- **`scale`/`offset`** transform raw sample heights; **`thickness`** adds a cuboid below the lowest point to avoid tunneling and is **not** affected by scale/offset; **`bWrap`** non-zero tiles the field infinitely (`include/ode/collision.h:1218`).
- **`bCopyHeightData`** (array builders): non-zero copies the array internally; zero references it — then the source array must persist for the heightfield's lifetime (`include/ode/collision.h:1266`).
- **Bounds**: `dGeomHeightfieldDataSetBounds` must be set **before** binding; the AABB is generated once and not recomputed. Callback heightfields otherwise default to ±infinity bounds, hurting broadphase rejection (`include/ode/collision.h:1436`).
- **Lifecycle**: create data → one `Build*` → `dCreateHeightfield(space, data, bPlaceable)`; destroy the geom first, then `dGeomHeightfieldDataDestroy` (`include/ode/collision.h:1147`).

## Pitfalls

- Calling `dGeomHeightfieldDataSetBounds` *after* binding is ignored — the AABB is fixed at first generation, so set bounds before `dCreateHeightfield` (`include/ode/collision.h:1436`).
- Build with `bCopyHeightData=0` from a stack-local array that goes out of scope → use-after-free during collision; either copy (`=1`) or keep the array alive (mistakes in trimesh-heightfield.json).
- Height is along **local Y**, not Z — assuming Z-up without rotating the placeable geom puts the terrain in the wrong plane (`include/ode/collision.h:1147`).
- `dGeomHeightfieldDataBuildSingle` takes `float*` and `dGeomHeightfieldDataBuildDouble` takes `double*` regardless of `dReal` — precision is selected by function name (`include/ode/collision.h:1364`,`:1414`).

## Never invent

- `dGeomHeightfieldDataBuildFloat` — the single-precision call is `dGeomHeightfieldDataBuildSingle`; header doc comments may say "BuildFloat" but no such symbol is exported (`include/ode/collision.h:1364`).
- `dCreateHeightfieldData` — the data allocator is `dGeomHeightfieldDataCreate`.

## Heightfield data API

The full data-build API (precision selection, scale/offset/thickness/wrap semantics, the `dHeightfieldGetHeight` callback typedef, and the data-outlives-geom rule) is documented in `references/trimesh-heightfield.md`.

See also the geoms overview: `references/geoms-and-spaces.md`.

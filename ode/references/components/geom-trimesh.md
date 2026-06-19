# Geom: Trimesh  (`dCreateTriMesh`)

> Source of truth: include/ode/collision_trimesh.h:253. Cited by file:line; do not invent symbols.

## What it is

A triangle-mesh collision geom built from caller-owned vertex and index arrays. You first build a **`dTriMeshDataID`** (which holds vertex pointers plus the bounding-volume tree), then create one or more geoms from it. The **data, and the vertex/index arrays it references, must outlive every geom built from it** (`include/ode/collision_trimesh.h:253`).

## Create & attach

Lifecycle: create data → one `dGeomTriMeshDataBuild*` → (optional `Preprocess2`) → `dCreateTriMesh`. **Vertex/index arrays are referenced, not copied** (`include/ode/collision_trimesh.h:120`).

```c
static float     Vertices[VertexCount][3];
static dTriIndex Indices[IndexCount];     /* 3 indices per triangle */

dTriMeshDataID Data = dGeomTriMeshDataCreate ();
/* Stride with sizeof, never magic 12/4 — dTriIndex may be 16-bit. */
dGeomTriMeshDataBuildSingle (Data,
        Vertices[0], 3 * sizeof(float), VertexCount,
        &Indices[0], IndexCount, 3 * sizeof(dTriIndex));
dGeomTriMeshDataPreprocess2 (Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);

dGeomID TriMesh = dCreateTriMesh (space, Data, 0, 0, 0);   /* callbacks optional (0/NULL) */
/* dGeomSetBody(TriMesh, body); for a moving mesh */
/* Shutdown: dGeomDestroy(TriMesh); THEN dGeomTriMeshDataDestroy(Data); */
```

Verbatim shape from `ode/demo/demo_trimesh.cpp:564-567`:

```c
dGeomTriMeshDataBuildSingle(Data, Vertices[0], 3 * sizeof(float), VertexCount, &Indices[0], IndexCount, 3 * sizeof(dTriIndex));
dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);
TriMesh = dCreateTriMesh(space, Data, 0, 0, 0);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateTriMesh` | `dGeomID dCreateTriMesh (dSpaceID space, dTriMeshDataID Data, dTriCallback* Callback, dTriArrayCallback* ArrayCallback, dTriRayCallback* RayCallback)` | include/ode/collision_trimesh.h:253 | Create a trimesh geom from built data; the three callbacks are optional (0/NULL). |
| `dGeomTriMeshDataCreate` | `dTriMeshDataID dGeomTriMeshDataCreate (void)` | include/ode/collision_trimesh.h:64 | Allocate an empty data object (you own it). |
| `dGeomTriMeshDataDestroy` | `void dGeomTriMeshDataDestroy (dTriMeshDataID g)` | include/ode/collision_trimesh.h:65 | Free the data — only after all geoms using it are destroyed. |
| `dGeomTriMeshDataBuildSingle` | `void dGeomTriMeshDataBuildSingle (dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride)` | include/ode/collision_trimesh.h:120 | Build from `float[3]` vertices (single precision). |
| `dGeomTriMeshDataBuildDouble` | `void dGeomTriMeshDataBuildDouble (dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride)` | include/ode/collision_trimesh.h:131 | Build from `double[3]` vertices (double precision). |
| `dGeomTriMeshDataBuildSimple` | `void dGeomTriMeshDataBuildSimple (dTriMeshDataID g, const dReal* Vertices, int VertexCount, const dTriIndex* Indices, int IndexCount)` | include/ode/collision_trimesh.h:143 | Build from `dReal` vertices; picks single/double from `dSINGLE`/`dDOUBLE`. |
| `dGeomTriMeshDataPreprocess2` | `int dGeomTriMeshDataPreprocess2 (dTriMeshDataID g, unsigned int buildRequestFlags, const dintptr *requestExtraData)` | include/ode/collision_trimesh.h:195 | Precompute concave-edge / face-angle data; run after build, before geom create. |
| `dGeomTriMeshSetData` | `void dGeomTriMeshSetData (dGeomID g, dTriMeshDataID Data)` | include/ode/collision_trimesh.h:255 | Rebind the geom to different (already-built) data. |
| `dGeomTriMeshGetData` | `dTriMeshDataID dGeomTriMeshGetData (dGeomID g)` | include/ode/collision_trimesh.h:256 | Read the data bound to the geom. |
| `dGeomTriMeshSetLastTransform` | `void dGeomTriMeshSetLastTransform (dGeomID g, const dMatrix4 last_trans)` | include/ode/collision_trimesh.h:114 | Push the previous-step transform (per geom) for accurate moving-mesh contacts. |
| `dGeomTriMeshSetCallback` | `void dGeomTriMeshSetCallback (dGeomID g, dTriCallback* Callback)` | include/ode/collision_trimesh.h:217 | Set the per-triangle filter callback (non-zero = accept). |
| `dGeomTriMeshGetTriangleCount` | `int dGeomTriMeshGetTriangleCount (dGeomID g)` | include/ode/collision_trimesh.h:307 | Number of triangles in the geom. |
| `dGeomTriMeshGetTriangle` | `void dGeomTriMeshGetTriangle (dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2)` | include/ode/collision_trimesh.h:280 | Fetch a triangle's three world-space vertices. |

## Parameters / conventions

- **Build precision is selected by function name, not by `dReal`**: `BuildSingle` reads `float[3]`, `BuildDouble` reads `double[3]`; use `BuildSimple` only when your vertices are stored as `dReal` (`include/ode/collision_trimesh.h:120-145`).
- **Strides use `sizeof`**: `VertexStride = 3*sizeof(float)` (or `sizeof(double)`), `TriStride = 3*sizeof(dTriIndex)`. `dTriIndex` is `duint32` normally but `duint16` under `dTRIMESH_16BIT_INDICES` — never hardcode 12/4 (`include/ode/common.h:78-86`).
- **Preprocess**: run `dGeomTriMeshDataPreprocess2` after a `Build*` and before creating the geom — `FACE_ANGLES` aids trimesh-convex, `CONCAVE_EDGES` aids trimesh-capsule; returns 0 only on out-of-memory (`include/ode/collision_trimesh.h:195`).
- **Moving meshes**: call `dGeomTriMeshSetLastTransform(geom, last_trans)` once per step (after the new transform) so ODE can estimate per-triangle velocity; stored per geom, not per data (`include/ode/collision_trimesh.h:114`).

## Pitfalls

- Destroying the `dTriMeshDataID` before the geom (or letting the vertex/index arrays go out of scope) leaves the geom pointing at freed memory — crashes during collision. Destroy the geom first, then the data (`include/ode/collision_trimesh.h:64`).
- Passing `double[3]` data to `dGeomTriMeshDataBuildSingle` (or vice versa) produces garbage geometry — the function name, not `dReal`, decides how the bytes are read (mistakes in trimesh-heightfield.json).
- Hardcoding `TriStride = 12` mis-strides the index array when `dTriIndex` is 16-bit; always use `3 * sizeof(dTriIndex)` (`include/ode/common.h:78-86`).
- **Skipping `dGeomTriMeshDataPreprocess2` after the `Build*` call** leaves the face-angle / concave-edge data unbuilt, which makes trimesh-vs-primitive contacts (sphere/box/capsule resting on a mesh) jitter or punch through (field-observed; header does not state this). The modern form is `dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL)` — `FACE_ANGLES` aids trimesh-convex, `CONCAVE_EDGES` aids trimesh-capsule; pass `NULL` for `requestExtraData` to take the defaults (`include/ode/collision_trimesh.h:158-163,195`).
- **Triangle winding / normal direction matters.** ODE derives each triangle's normal from vertex order, and contacts are only reliable when that normal faces the colliding body — outward-facing for a surface a body rests *on*, inward-facing for a funnel/container that holds bodies in. A back-facing triangle yields unreliable sphere-vs-trimesh contacts (bodies sink in or skip off). The header is silent on the winding convention, so fix it empirically: if contacts are wrong, flip the index order of the offending triangles (field-observed; not stated in `include/ode/collision_trimesh.h`).

## Never invent

- `dGeomTriMeshDataBuildNormals` or a single-call "build+create" helper — the only build entrypoints are `BuildSingle/Single1/Double/Double1/Simple/Simple1` (`include/ode/collision_trimesh.h:120-150`).
- `dCreateTriMesh` is **not** in `collision.h` — it lives in `collision_trimesh.h`; the geom class is `dTriMeshClass`.

## Trimesh data API

The full data lifecycle, build-precision rules, `dTriIndex` width contract, callbacks (`dTriCallback`/`dTriArrayCallback`/`dTriRayCallback`/`dTriTriMergeCallback`), preprocess flags, and moving-mesh transform handling are documented in `references/trimesh-heightfield.md`.

See also the geoms overview: `references/geoms-and-spaces.md`.

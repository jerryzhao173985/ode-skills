# Trimesh, Heightfield & Convex collision geoms

> Source of truth: include/ode/collision_trimesh.h and include/ode/collision.h (with include/ode/common.h for dTriIndex). Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model
- A trimesh has two objects: the **data** (`dTriMeshDataID`, the vertex pointers + BV tree) and the **geom** (`dGeomID`). One data object can be shared by several geoms. (include/ode/collision_trimesh.h:26-27, :44-45)
- **The data must OUTLIVE its geoms.** You own the `dTriMeshDataID`/`dHeightfieldDataID`, not ODE; vertex/index/height arrays you pass are *referenced, not copied*. Destroy geoms first, then the data. (include/ode/collision_trimesh.h:64-65, :100-102)
- **Build precision is chosen by the function NAME, not by `dReal`.** `dGeomTriMeshDataBuildSingle` reads `float[3]`; `BuildDouble` reads `double[3]`; only `BuildSimple` follows `dSINGLE`/`dDOUBLE`. (include/ode/collision_trimesh.h:118-141)
- A **heightfield** is an efficient regular-grid terrain: height comes along the geom's **local Y** axis with the field in the local x/z plane (rotate it for a Z-up world). (include/ode/collision.h:1190-1194)
- A **convex** geom is a hull described by face planes + a point cloud + a polygon index list; those arrays are also referenced and must outlive the geom. (include/ode/collision.h:965-970)

## When to use
- Use a **trimesh** for arbitrary triangle-soup geometry (imported models, static level meshes, moving concave shapes). Prefer simpler primitives (box/sphere/capsule) when they suffice — they collide faster.
- Use a **heightfield** for terrain / a 2.5-D regular grid of heights: far cheaper than a trimesh of the same surface and supports infinite tiling (`bWrap`). (include/ode/collision.h:1216-1218)
- Use a **convex** geom when you have a true convex hull (planes + points known) and want exact convex-convex contacts without a full trimesh.

## Public API (C)

### Trimesh data: lifecycle & build
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomTriMeshDataCreate` | `dTriMeshDataID dGeomTriMeshDataCreate(void);` | include/ode/collision_trimesh.h:64 | Allocate an empty trimesh data object; fill it with a `Build*` call before binding to a geom. |
| `dGeomTriMeshDataDestroy` | `void dGeomTriMeshDataDestroy(dTriMeshDataID g);` | include/ode/collision_trimesh.h:65 | Free the data. You own it: destroy only after every geom built from it is gone. |
| `dGeomTriMeshDataBuildSingle` | `void dGeomTriMeshDataBuildSingle(dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride);` | include/ode/collision_trimesh.h:120 | Build from **single-precision** `float[3]` vertices; arrays referenced not copied. |
| `dGeomTriMeshDataBuildSingle1` | `void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride, const void* Normals);` | include/ode/collision_trimesh.h:124 | As `BuildSingle` plus a precomputed per-triangle `Normals` array (trimesh-trimesh optimization). |
| `dGeomTriMeshDataBuildDouble` | `void dGeomTriMeshDataBuildDouble(dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride);` | include/ode/collision_trimesh.h:131 | Build from **double-precision** `double[3]` vertices; same stride/index semantics. |
| `dGeomTriMeshDataBuildDouble1` | `void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g, const void* Vertices, int VertexStride, int VertexCount, const void* Indices, int IndexCount, int TriStride, const void* Normals);` | include/ode/collision_trimesh.h:135 | Double-precision build with a per-triangle `Normals` array. |
| `dGeomTriMeshDataBuildSimple` | `void dGeomTriMeshDataBuildSimple(dTriMeshDataID g, const dReal* Vertices, int VertexCount, const dTriIndex* Indices, int IndexCount);` | include/ode/collision_trimesh.h:143 | Convenience build; picks single/double from `dSINGLE`/`dDOUBLE`. Flat `dReal[3]` verts, flat `dTriIndex` indices, no stride args. |
| `dGeomTriMeshDataBuildSimple1` | `void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g, const dReal* Vertices, int VertexCount, const dTriIndex* Indices, int IndexCount, const int* Normals);` | include/ode/collision_trimesh.h:147 | `dReal`-precision Simple build with a per-triangle normals index array. |
| `dGeomTriMeshDataPreprocess2` | `int dGeomTriMeshDataPreprocess2(dTriMeshDataID g, unsigned int buildRequestFlags, const dintptr *requestExtraData);` | include/ode/collision_trimesh.h:195 | Precompute edge/face-angle data. `buildRequestFlags` = bitmask of `1U << dTRIDATAPREPROCESS_BUILD_*`. Returns boolean; only failure is OOM. |
| `dGeomTriMeshDataPreprocess` | `int dGeomTriMeshDataPreprocess(dTriMeshDataID g);` | include/ode/collision_trimesh.h:200 | Obsolete one-arg form = `Preprocess2(g, 1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES, NULL)`. Prefer the `*2` form. |
| `dGeomTriMeshDataUpdate` | `void dGeomTriMeshDataUpdate(dTriMeshDataID g);` | include/ode/collision_trimesh.h:309 | Notify ODE the referenced vertex data changed in place so it rebuilds cached BV structure (deformable/animated meshes). |

### Trimesh data: auxiliary data & buffers
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomTriMeshDataSet` | `void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void *in_data);` | include/ode/collision_trimesh.h:103 | Attach aux data keyed by `data_id` (`dTRIMESHDATA_FACE_NORMALS`/`dTRIMESHDATA_USE_FLAGS`). **NOT copied** — must outlive the data. |
| `dGeomTriMeshDataGet` | `void *dGeomTriMeshDataGet(dTriMeshDataID g, int data_id);` | include/ode/collision_trimesh.h:104 | Retrieve aux data previously set for `data_id`. |
| `dGeomTriMeshDataGet2` | `void *dGeomTriMeshDataGet2(dTriMeshDataID g, int data_id, dsizeint *pout_size);` | include/ode/collision_trimesh.h:105 | Like `Get` but also returns the block size via `pout_size` (NULL to ignore). Preferred way to read `dTRIMESHDATA_USE_FLAGS`. |
| `dGeomTriMeshDataGetBuffer` | `ODE_API_DEPRECATED void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char** buf, int* bufLen);` | include/ode/collision_trimesh.h:208 | **DEPRECATED.** Read back preprocessed buffer. Use `dGeomTriMeshDataGet2` + `dTRIMESHDATA_USE_FLAGS`. |
| `dGeomTriMeshDataSetBuffer` | `ODE_API_DEPRECATED void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf);` | include/ode/collision_trimesh.h:209 | **DEPRECATED.** Load a saved preprocessed buffer. Use `dGeomTriMeshDataSet` + `dTRIMESHDATA_USE_FLAGS`. |

### Trimesh geom: create, bind, transforms
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateTriMesh` | `dGeomID dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback* Callback, dTriArrayCallback* ArrayCallback, dTriRayCallback* RayCallback);` | include/ode/collision_trimesh.h:253 | Create a trimesh geom from a built `Data`. Callbacks optional (pass 0). `Data` must stay valid for the geom's whole life. |
| `dGeomTriMeshSetData` | `void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data);` | include/ode/collision_trimesh.h:255 | Rebind the geom to a different (already built) data object that outlives the geom. |
| `dGeomTriMeshGetData` | `dTriMeshDataID dGeomTriMeshGetData(dGeomID g);` | include/ode/collision_trimesh.h:256 | Return the data currently bound to the geom. |
| `dGeomTriMeshGetTriMeshDataID` | `dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g);` | include/ode/collision_trimesh.h:275 | Alias-style accessor for the bound `dTriMeshDataID`. |
| `dGeomTriMeshSetLastTransform` | `void dGeomTriMeshSetLastTransform(dGeomID g, const dMatrix4 last_trans);` | include/ode/collision_trimesh.h:114 | Store the previous-step transform (per geom instance) so ODE estimates per-triangle velocity for accurate moving-mesh contacts. |
| `dGeomTriMeshGetLastTransform` | `const dReal* dGeomTriMeshGetLastTransform(dGeomID g);` | include/ode/collision_trimesh.h:115 | Return the last transform (4x4, `dMatrix4` layout) set on the geom. |
| `dGeomTriMeshGetTriangle` | `void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2);` | include/ode/collision_trimesh.h:280 | Fetch the three world-space vertices of triangle `Index` (drawing/debug inside callbacks). |
| `dGeomTriMeshGetPoint` | `void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out);` | include/ode/collision_trimesh.h:286 | Compute the world point on triangle `Index` at barycentric `(u,v)` — pairs with `dTriRayCallback`. |
| `dGeomTriMeshGetTriangleCount` | `int dGeomTriMeshGetTriangleCount(dGeomID g);` | include/ode/collision_trimesh.h:307 | Number of triangles in the geom. |

### Trimesh callbacks (all optional)
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomTriMeshSetCallback` | `void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback);` | include/ode/collision_trimesh.h:217 | Per-triangle filter (`dTriCallback`): return non-zero to allow, zero to skip the triangle. |
| `dGeomTriMeshGetCallback` | `dTriCallback* dGeomTriMeshGetCallback(dGeomID g);` | include/ode/collision_trimesh.h:218 | Return the per-triangle callback. |
| `dGeomTriMeshSetArrayCallback` | `void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback);` | include/ode/collision_trimesh.h:225 | Per-object callback (`dTriArrayCallback`): receive the full candidate-triangle list at once. |
| `dGeomTriMeshGetArrayCallback` | `dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g);` | include/ode/collision_trimesh.h:226 | Return the array callback. |
| `dGeomTriMeshSetRayCallback` | `void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback);` | include/ode/collision_trimesh.h:235 | Ray-collision callback (`dTriRayCallback`): accept/reject a ray-triangle hit using barycentric `(u,v)`. |
| `dGeomTriMeshGetRayCallback` | `dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g);` | include/ode/collision_trimesh.h:236 | Return the ray callback. |
| `dGeomTriMeshSetTriMergeCallback` | `void dGeomTriMeshSetTriMergeCallback(dGeomID g, dTriTriMergeCallback* Callback);` | include/ode/collision_trimesh.h:246 | Triangle-merge callback (`dTriTriMergeCallback`): synthesize a triangle index when two contacts merge. |
| `dGeomTriMeshGetTriMergeCallback` | `dTriTriMergeCallback* dGeomTriMeshGetTriMergeCallback(dGeomID g);` | include/ode/collision_trimesh.h:247 | Return the triangle-merge callback. |

### Trimesh temporal coherence (TC)
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomTriMeshEnableTC` | `void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable);` | include/ode/collision_trimesh.h:260 | Enable/disable temporal-coherence caching vs a geom class (e.g. `dSphereClass`, `dBoxClass`, `dCapsuleClass`). |
| `dGeomTriMeshIsTCEnabled` | `int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass);` | include/ode/collision_trimesh.h:261 | Query whether TC is enabled for that geom class. |
| `dGeomTriMeshClearTCCache` | `void dGeomTriMeshClearTCCache(dGeomID g);` | include/ode/collision_trimesh.h:269 | Clear the trimesh's internal TC caches; bound memory in large worlds. |

### Heightfield (include/ode/collision.h)
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomHeightfieldDataCreate` | `dHeightfieldDataID dGeomHeightfieldDataCreate(void);` | include/ode/collision.h:1163 | Allocate an empty heightfield data object; configure with a `Build*` call before use. You own it. |
| `dGeomHeightfieldDataDestroy` | `void dGeomHeightfieldDataDestroy(dHeightfieldDataID d);` | include/ode/collision.h:1174 | Free the data and its managed resources; destroy only after all geoms using it are gone. |
| `dGeomHeightfieldDataBuildCallback` | `void dGeomHeightfieldDataBuildCallback(dHeightfieldDataID d, void* pUserData, dHeightfieldGetHeight* pCallback, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap);` | include/ode/collision.h:1218 | Sample heights via a `dHeightfieldGetHeight` callback (procedural terrain). Defaults to +/- INF bounds — set bounds explicitly. |
| `dGeomHeightfieldDataBuildByte` | `void dGeomHeightfieldDataBuildByte(dHeightfieldDataID d, const unsigned char* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap);` | include/ode/collision.h:1266 | Row-major array of 8-bit unsigned heights. `bCopyHeightData` non-zero copies; zero references (must persist). |
| `dGeomHeightfieldDataBuildShort` | `void dGeomHeightfieldDataBuildShort(dHeightfieldDataID d, const short* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap);` | include/ode/collision.h:1314 | Row-major array of 16-bit signed shorts; same copy/reference and extent semantics. |
| `dGeomHeightfieldDataBuildSingle` | `void dGeomHeightfieldDataBuildSingle(dHeightfieldDataID d, const float* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap);` | include/ode/collision.h:1364 | Row-major **`float*`** heights regardless of `dReal`. (This is the single-precision call — there is NO `dGeomHeightfieldDataBuildFloat`.) |
| `dGeomHeightfieldDataBuildDouble` | `void dGeomHeightfieldDataBuildDouble(dHeightfieldDataID d, const double* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap);` | include/ode/collision.h:1414 | Row-major **`double*`** heights regardless of `dReal`. |
| `dGeomHeightfieldDataSetBounds` | `void dGeomHeightfieldDataSetBounds(dHeightfieldDataID d, dReal minHeight, dReal maxHeight);` | include/ode/collision.h:1436 | Set min/max height bounds for the AABB. **Must be set before binding** — AABB is generated once and never recomputed. |
| `dCreateHeightfield` | `dGeomID dCreateHeightfield(dSpaceID space, dHeightfieldDataID data, int bPlaceable);` | include/ode/collision.h:1147 | Create the heightfield geom. `bPlaceable` non-zero ⇒ movable/rotatable; else fixed with global y = height. Data must outlive the geom. |
| `dGeomHeightfieldSetHeightfieldData` | `void dGeomHeightfieldSetHeightfieldData(dGeomID g, dHeightfieldDataID d);` | include/ode/collision.h:1450 | Bind a data object to an existing heightfield geom without touching its `GEOM_PLACEABLE` flag. |
| `dGeomHeightfieldGetHeightfieldData` | `dHeightfieldDataID dGeomHeightfieldGetHeightfieldData(dGeomID g);` | include/ode/collision.h:1462 | Return the bound data (may be NULL if none assigned). |

### Convex (include/ode/collision.h)
| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateConvex` | `dGeomID dCreateConvex(dSpaceID space, const dReal *_planes, unsigned int _planecount, const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons);` | include/ode/collision.h:965 | Convex-hull geom from face planes (`_planes`: 4 dReals a,b,c,d each), points (`_points`: xyz triples), and a polygon index list. Arrays referenced — must outlive the geom. |
| `dGeomSetConvex` | `void dGeomSetConvex(dGeomID g, const dReal *_planes, unsigned int _count, const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons);` | include/ode/collision.h:972 | Re-set an existing convex geom's data (same plane/point/polygon layout). |

### Types & callbacks
| Symbol | Definition | Source | Purpose |
| --- | --- | --- | --- |
| `dTriMeshDataID` | `typedef struct dxTriMeshData* dTriMeshDataID;` | include/ode/collision_trimesh.h:44-45 | Opaque handle: vertex pointers + BV tree. Shared by geoms; must outlive them. |
| `dHeightfieldDataID` | `typedef struct dxHeightfieldData* dHeightfieldDataID;` | include/ode/collision.h:1103-1104 | Opaque handle: extents, sample grid, scale/offset/thickness/wrap, bounds. Must outlive its geom. |
| `dHeightfieldGetHeight` | `typedef dReal dHeightfieldGetHeight(void* p_user_data, int x, int z);` | include/ode/collision.h:1124 | Returns the height at integer grid `(x,z)`; `p_user_data` is the `pUserData` from `BuildCallback`. |
| `dTriCallback` | `typedef int dTriCallback(dGeomID TriMesh, dGeomID RefObject, int TriangleIndex);` | include/ode/collision_trimesh.h:216 | Per-triangle filter: non-zero accept, zero skip. |
| `dTriArrayCallback` | `typedef void dTriArrayCallback(dGeomID TriMesh, dGeomID RefObject, const int* TriIndices, int TriCount);` | include/ode/collision_trimesh.h:224 | Delivers all candidate triangle indices in one call. |
| `dTriRayCallback` | `typedef int dTriRayCallback(dGeomID TriMesh, dGeomID Ray, int TriangleIndex, dReal u, dReal v);` | include/ode/collision_trimesh.h:234 | Accept/reject ray-vs-triangle using barycentric `(u,v)`. |
| `dTriTriMergeCallback` | `typedef int dTriTriMergeCallback(dGeomID TriMesh, int FirstTriangleIndex, int SecondTriangleIndex);` | include/ode/collision_trimesh.h:245 | Returns a synthetic triangle index for a contact merged from two source triangles. |
| `dMeshTriangleVertex` | `enum { dMTV__MIN, dMTV_FIRST = dMTV__MIN, dMTV_SECOND, dMTV_THIRD, dMTV__MAX };` | include/ode/collision_trimesh.h:48-58 | Names the three vertices of a triangle. |
| `dTriIndex` | `typedef duint32 dTriIndex;` (or `duint16` under `dTRIMESH_16BIT_INDICES` unless `dTRIMESH_GIMPACT`) | include/ode/common.h:78-86 | Index integer type for `Indices` arrays. Size strides with `sizeof(dTriIndex)`, never a hardcoded 4. |

## Key rules
- A `dTriMeshDataID`/`dHeightfieldDataID` is owned by you, not ODE, and must remain alive for the whole lifetime of every geom built from it; destroy the geom first, then the data (include/ode/collision_trimesh.h:64-65), and on shutdown call `dGeomHeightfieldDataDestroy` because "_we_ own it not ODE" (ode/demo/demo_heightfield.cpp:710).
- Vertex/index arrays passed to `Build*` are referenced, NOT copied — they must outlive the data; likewise "Note: The data is NOT COPIED on assignment" for `dGeomTriMeshDataSet` (include/ode/collision_trimesh.h:100-103).
- Match the build-precision function to your data, not to `dReal`: `BuildSingle` expects `float[3]`, `BuildDouble` expects `double[3]`; use `BuildSimple` (single/double from `dSINGLE`/`dDOUBLE`) only when vertices are stored as `dReal`; the header warns "Vertices should be single precision!" (include/ode/collision_trimesh.h:28).
- Size strides with `sizeof`: `VertexStride = 3 * sizeof(float)` (or `sizeof(double)`), `TriStride = 3 * sizeof(dTriIndex)` — never hardcode 12 or 4 (ode/demo/demo_trimesh.cpp:564).
- `dTriIndex` is `duint16` when ODE is built with `dTRIMESH_16BIT_INDICES` (non-GIMPACT) and `duint32` otherwise, so always derive strides from `sizeof(dTriIndex)` (include/ode/common.h:78-86).
- For moving trimeshes, call `dGeomTriMeshSetLastTransform(geom, last_trans)` once per step after computing the new transform so ODE estimates per-triangle velocity; this is stored per geom instance, not per data (include/ode/collision_trimesh.h:110-112).
- Run `dGeomTriMeshDataPreprocess2` after a `Build*` call; pass `1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES` for trimesh-convex and/or `CONCAVE_EDGES` for trimesh-capsule; it returns 0 only on out-of-memory (include/ode/collision_trimesh.h:189-194), and the demos call it right before `dCreateTriMesh` (ode/demo/demo_trimesh.cpp:565).
- For heightfields, set explicit min/max bounds with `dGeomHeightfieldDataSetBounds` BEFORE binding — "the AABB is not recomputed after it's first generation" (include/ode/collision.h:1419-1422); the demo sets bounds before `dCreateHeightfield` (ode/demo/demo_heightfield.cpp:672-676).
- Heightfield height is the geom's LOCAL Y axis with the field in the local x/z plane (include/ode/collision.h:1190-1194); for a Z-up world create it placeable and rotate 90° about X (ode/demo/demo_heightfield.cpp:682-687).
- Heightfield `Build*`: `bCopyHeightData` non-zero copies the array internally, zero references it (the array must then persist for the heightfield's life); `thickness` is subtracted from the lowest height and is NOT affected by scale/offset (include/ode/collision.h:1235-1257).
- A callback heightfield defaults to +/- infinity bounds, so without `dGeomHeightfieldDataSetBounds` broadphase cannot reject it (include/ode/collision.h:1419-1421).
- `widthSamples`/`depthSamples` are vertex counts and "must be at least two or more" (include/ode/collision.h:1197-1200).

## Common mistakes
| Bad | Good | Why |
| --- | --- | --- |
| `dGeomTriMeshDataDestroy(data);` right after `dCreateTriMesh`, before stepping. | Keep the `dTriMeshDataID` alive the whole sim; destroy it only after `dGeomDestroy(g)` / at shutdown. | The geom only references the data + BV tree inside it; destroying it early leaves the geom pointing at freed memory (include/ode/collision_trimesh.h:64-65). |
| `double verts[][3]={...}; dGeomTriMeshDataBuildSingle(data, verts, 3*sizeof(double), ...);` | Use `dGeomTriMeshDataBuildDouble` for `double[3]` (or store verts as `float` + `BuildSingle`). | `BuildSingle` reinterprets bytes as `float[3]`; the function name — not `dReal` — selects how vertex bytes are read (include/ode/collision_trimesh.h:118-120). |
| `int TriStride = 12; // 3 * 4-byte indices` | `int TriStride = 3 * sizeof(dTriIndex);` and cast index pointers to `dTriIndex*`. | `dTriIndex` is `duint16` under `dTRIMESH_16BIT_INDICES` (non-GIMPACT), so a hardcoded 12/4 silently mis-strides the index array (include/ode/common.h:78-86). |
| Create the callback heightfield, bind it, then call `dGeomHeightfieldDataSetBounds` afterward. | Call `dGeomHeightfieldDataSetBounds` BEFORE `dCreateHeightfield` / `dGeomHeightfieldSetHeightfieldData`. | The AABB is generated once at first binding and "not recomputed after it's first generation," so later bounds are ignored (include/ode/collision.h:1419-1422). |
| Build a heightfield with `bCopyHeightData=0` from a stack-local height array that goes out of scope. | Pass `bCopyHeightData=1` to copy, or keep the array alive as long as the heightfield. | With `bCopyHeightData` zero the data "is accessed by reference and so must persist throughout the lifetime of the heightfield" (include/ode/collision.h:1252-1254). |
| Assume `dGeomHeightfieldDataBuildFloat` is the single-precision call (it appears in doc comments). | Use `dGeomHeightfieldDataBuildSingle`. | No `BuildFloat` symbol is declared; only the doc comments mention the name (include/ode/collision.h:1364). |

## Named constants
| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dTRIMESHDATA_FACE_NORMALS` | enum (= `dTRIMESHDATA__MIN`) | include/ode/collision_trimesh.h:75 | `data_id` for `Set/Get` to attach a precomputed per-face normals array. (Legacy alias `TRIMESH_FACE_NORMALS` == this, kept for back-compat, collision_trimesh.h:82.) |
| `dTRIMESHDATA_USE_FLAGS` | enum (next after `FACE_NORMALS`) | include/ode/collision_trimesh.h:76 | `data_id` for `Set`/`Get2` to attach/read the per-triangle edge/vertex use-flags buffer (replaces deprecated Get/SetBuffer). |
| `dMESHDATAUSE_EDGE1` | `0x01` | include/ode/collision_trimesh.h:91 | Use-flag bit: triangle edge 1 participates in collision. |
| `dMESHDATAUSE_EDGE2` | `0x02` | include/ode/collision_trimesh.h:92 | Use-flag bit: triangle edge 2. |
| `dMESHDATAUSE_EDGE3` | `0x04` | include/ode/collision_trimesh.h:93 | Use-flag bit: triangle edge 3. |
| `dMESHDATAUSE_VERTEX1` | `0x08` | include/ode/collision_trimesh.h:94 | Use-flag bit: triangle vertex 1. |
| `dMESHDATAUSE_VERTEX2` | `0x10` | include/ode/collision_trimesh.h:95 | Use-flag bit: triangle vertex 2. |
| `dMESHDATAUSE_VERTEX3` | `0x20` | include/ode/collision_trimesh.h:96 | Use-flag bit: triangle vertex 3. |
| `dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES` | enum (= `dTRIDATAPREPROCESS_BUILD__MIN`) | include/ode/collision_trimesh.h:160 | Preprocess flag (use as `1U <<`). Optimizes OPCODE trimesh-capsule; 1 byte/triangle, no extra data. |
| `dTRIDATAPREPROCESS_BUILD_FACE_ANGLES` | enum (next) | include/ode/collision_trimesh.h:161 | Preprocess flag (use as `1U <<`). Aids trimesh-convex; memory depends on the FACE_ANGLES extra mode. Used by the trimesh demos. |
| `dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_POSITIVE` | enum (= `__MIN`, the `__DEFAULT`) | include/ode/collision_trimesh.h:173 | FACE_ANGLES extra mode: convex edges only, 3 bytes/triangle (default if extra data omitted). |
| `dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_BYTE_ALL` | enum (next) | include/ode/collision_trimesh.h:174 | FACE_ANGLES extra mode: all edges, 3 bytes/triangle. |
| `dTRIDATAPREPROCESS_FACE_ANGLES_EXTRA_WORD_ALL` | enum (next) | include/ode/collision_trimesh.h:175 | FACE_ANGLES extra mode: all edges, 6 bytes/triangle (finest resolution). |

## Code: static trimesh — build single-precision data, preprocess, create geom (data must outlive the geom)
Source: ode/demo/demo_trimesh.cpp:119-124, 561-567

```c
#define VertexCount 5
#define IndexCount  12
static float     Vertices[VertexCount][3];
static dTriIndex Indices[IndexCount];
// ... fill Vertices[] and Indices[] (3 indices per triangle) ...

// 1) Allocate the data object (you own it, not ODE).
dTriMeshDataID Data = dGeomTriMeshDataCreate();

// 2) Build from SINGLE-precision data. Stride with sizeof, not magic numbers.
//    Vertices/Indices are referenced, not copied -> they must outlive Data.
dGeomTriMeshDataBuildSingle(Data,
        Vertices[0], 3 * sizeof(float), VertexCount,
        &Indices[0], IndexCount, 3 * sizeof(dTriIndex));

// 3) Precompute face angles (aids trimesh-convex); call before creating the geom.
dGeomTriMeshDataPreprocess2(Data, (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);

// 4) Create the geom. Callbacks are optional (0/NULL). Keep Data alive for its lifetime.
dGeomID TriMesh = dCreateTriMesh(space, Data, 0, 0, 0);

// ... simulate ...
// On shutdown: destroy the geom first, then the data.
// dGeomDestroy(TriMesh);
// dGeomTriMeshDataDestroy(Data);
```

## Code: moving trimesh — push the previous transform each step for accurate contacts
Source: ode/demo/demo_moving_trimesh.cpp:459-475, 494-497

```c
void setCurrentTransform(dGeomID geom)
{
    const dReal* Pos = dGeomGetPosition(geom);
    const dReal* Rot = dGeomGetRotation(geom);
    const dReal Transform[16] = {
        Rot[0], Rot[4], Rot[8],  0,
        Rot[1], Rot[5], Rot[9],  0,
        Rot[2], Rot[6], Rot[10], 0,
        Pos[0], Pos[1], Pos[2],  1
    };
    // Store as the geom's 'last' transform; ODE uses it next collision step.
    dGeomTriMeshSetLastTransform(geom, *(dMatrix4*)(&Transform));
}
// ... after each dWorldStep, before the next collision pass, for every moving trimesh geom:
//     setCurrentTransform(triMeshGeom);
```

## Code: finite callback heightfield with explicit bounds, rotated for Z-up
Source: ode/demo/demo_heightfield.cpp:156, 665-690, 710

```c
dReal heightfield_callback(void*, int x, int z) { /* return terrain height at grid (x,z) */ }

dHeightfieldDataID heightid = dGeomHeightfieldDataCreate();

// Finite (bWrap=0) callback field; scale=1, offset=0, thickness=0.
dGeomHeightfieldDataBuildCallback(heightid, NULL, heightfield_callback,
        HFIELD_WIDTH, HFIELD_DEPTH, HFIELD_WSTEP, HFIELD_DSTEP,
        REAL(1.0), REAL(0.0), REAL(0.0), 0);

// Set bounds BEFORE binding: AABB is generated once and never recomputed.
dGeomHeightfieldDataSetBounds(heightid, REAL(-4.0), REAL(+6.0));

// bPlaceable=1 so we can position/rotate it.
dGeomID gheight = dCreateHeightfield(space, heightid, 1);

// Height is along local Y by default; rotate 90deg about X for a Z-up world.
dMatrix3 R; dRSetIdentity(R);
dRFromAxisAndAngle(R, 1, 0, 0, DEGTORAD * 90);
dGeomSetRotation(gheight, R);
dGeomSetPosition(gheight, 0, 0, 0);
// On shutdown: dGeomHeightfieldDataDestroy(heightid); // we own it, not ODE
```

## Things to never invent
- Do not invent a `dGeomTriMeshDataBuildNormals` or a single-call "build+create" helper — the only build entrypoints are `BuildSingle`/`Single1`/`Double`/`Double1`/`Simple`/`Simple1` (include/ode/collision_trimesh.h:120-147).
- Do not claim `dGeomTriMeshDataBuildSingle` copies the vertex/index arrays — they are referenced (include/ode/collision_trimesh.h:100-103).
- Do not assume `dTriIndex` is always 32-bit or 4 bytes; it can be `duint16` (include/ode/common.h:78-86).
- Do not invent a `dGeomHeightfieldDataBuildFloat` name — the single-precision call is `dGeomHeightfieldDataBuildSingle` (include/ode/collision.h:1364); the header doc comments say "BuildFloat" but no such symbol is declared.
- Do not claim `dGeomHeightfieldDataSetBounds` can be applied after the geom is bound; the AABB is fixed at first generation (include/ode/collision.h:1419-1422).
- Do not state heightfield height is along Z by default — it is along the local Y axis (include/ode/collision.h:1190-1194).

## [VERIFY] — facts not fully grounded in the headers read
- Exact memory layout / winding convention required by `dCreateConvex`'s `_planes`/`_points`/`_polygons` arrays: only the signature is in include/ode/collision.h:965-970; the full contract lives in the convex backend / demo_convex.cpp (not read here). [VERIFY]
- `dGeomTriMeshDataUpdate` semantics for deformable meshes: the header (include/ode/collision_trimesh.h:309) only says it updates the data; whether it fully rebuilds the BV tree vs. incrementally is a backend detail not confirmed from headers. [VERIFY]

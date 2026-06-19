# Geoms and spaces: the collision-shape catalog, placeable binding, and broadphase

> Source of truth: include/ode/collision.h, include/ode/collision_space.h, include/ode/contact.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

This is the **geom-creation + spaces** half of collision. The `dCollide`/`dSpaceCollide` near-callback **contact pipeline** (turning candidate pairs into contact joints, `dContact`/`dSurfaceParameters`, `dJointGroupEmpty`) lives in references/collision-and-contacts.md — cross-link, do not duplicate. Per-shape setters/getters (`dGeomSphereSetRadius`, `dGeomBoxGetLengths`, `dGeomRayGet`, trimesh data, …) live in the component files linked in the catalog below.

## Mental model

- **A geom is a shape with no mass; a body is mass with no shape.** They are independent objects, linked only by `dGeomSetBody` so they share a pose (include/ode/collision.h:88). A dynamic object = body + geom(s) bound together.
- **A space is itself a geom** (`dGeomIsSpace` returns non-zero) and a broadphase container. It does not store mass or get stepped; it just accelerates "which pairs are near each other" (include/ode/collision.h:270).
- **Placeable vs non-placeable.** Sphere/box/capsule/cylinder/ray/convex/trimesh (and a placeable heightfield) have a settable position+rotation. Plane and a fixed heightfield are non-placeable — they are defined by their equation/global frame and `dGeomSetPosition`/`dGeomSetRotation` is illegal on them (include/ode/collision.h:88).
- **Static geoms have body=0.** A geom with no body bound (e.g. the ground plane) reads back `dGeomGetBody == 0`, which the contact pipeline treats as the immovable environment (include/ode/collision.h:114).
- **Pose flows body → geom.** When a geom is attached to a body, moving the body moves the geom; `dGeomSetPosition`/`dGeomSetRotation` on an attached geom also moves the body (include/ode/collision.h:131).
- **Lifecycle is hierarchical.** Destroying a space (cleanup mode 1, the default) destroys every geom in it; `dGeomDestroy` removes a geom from its space first, and works for any geom type including a sub-space (include/ode/collision.h:59).

## When to use

- Building the collidable world: ground plane, walls, and one geom per dynamic body.
- Choosing a broadphase: simple (tiny scenes), hash (general default), quadtree (planar/static), sweep-and-prune (many moving bodies).
- Filtering collisions cheaply via category/collide bitfields or `dGeomDisable`, before the `dCollide` step.
- Offsetting a geom from its body's center (e.g. an off-center wheel) via `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation`.
- Nesting spaces (a sub-space per articulated object) so the broadphase skips internal pairs.

For converting the candidate pairs this module produces into actual contact forces, go to references/collision-and-contacts.md.

## Public API (C)

### Geom primitives (one creator per shape; each links to its component file)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateSphere` | `dGeomID dCreateSphere (dSpaceID space, dReal radius)` | include/ode/collision.h:921 | Sphere geom; reference point is its center; `space` may be null. See references/components/geom-sphere.md. |
| `dCreateBox` | `dGeomID dCreateBox (dSpaceID space, dReal lx, dReal ly, dReal lz)` | include/ode/collision.h:1001 | Box geom with **FULL side lengths** `lx,ly,lz` (not half-extents); reference point is its center. See references/components/geom-box.md. |
| `dCreatePlane` | `dGeomID dCreatePlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d)` | include/ode/collision.h:1045 | **Non-placeable** infinite plane `a*x+b*y+c*z=d` (static ground); `(a,b,c)` is the unit normal. See references/components/geom-plane.md. |
| `dCreateCapsule` | `dGeomID dCreateCapsule (dSpaceID space, dReal radius, dReal length)` | include/ode/collision.h:1050 | Capsule (round-ended cylinder); `length` is the cylinder section only (caps add `radius` each end). `dCreateCCylinder` is a `#define` alias. See references/components/geom-capsule.md. |
| `dCreateCylinder` | `dGeomID dCreateCylinder (dSpaceID space, dReal radius, dReal length)` | include/ode/collision.h:1062 | Flat-ended cylinder of given `radius` and `length`. See references/components/geom-cylinder.md. |
| `dCreateRay` | `dGeomID dCreateRay (dSpaceID space, dReal length)` | include/ode/collision.h:1066 | Ray geom of given `length`; set start+direction with `dGeomRaySet`. See references/components/geom-ray.md. |
| `dGeomRaySet` | `void dGeomRaySet (dGeomID ray, dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)` | include/ode/collision.h:1069 | Set a ray's start point `(px,py,pz)` and direction `(dx,dy,dz)`. See references/components/geom-ray.md. |
| `dCreateConvex` | `dGeomID dCreateConvex (dSpaceID space, const dReal *_planes, unsigned int _planecount, const dReal *_points, unsigned int _pointcount, const unsigned int *_polygons)` | include/ode/collision.h:965 | Convex geom from plane equations, vertices, and polygon index data. See references/components/geom-convex.md. |
| `dCreateGeomTransform` | `dGeomID dCreateGeomTransform (dSpaceID space)` | include/ode/collision.h:1089 | **DEPRECATED** wrapper that encapsulates another geom at a local offset; prefer `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation`. |

### Heightfield (data object + geom)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateHeightfield` | `dGeomID dCreateHeightfield (dSpaceID space, dHeightfieldDataID data, int bPlaceable)` | include/ode/collision.h:1147 | Heightfield geom backed by a configured `dHeightfieldDataID`; `bPlaceable!=0` enables `dGeomSetPosition`/`Rotation`, else fixed with global Y as height. See references/components/geom-heightfield.md. |
| `dGeomHeightfieldDataCreate` | `dHeightfieldDataID dGeomHeightfieldDataCreate (void)` | include/ode/collision.h:1163 | Allocate empty heightfield data; must be configured by a `Build*` call before use. |
| `dGeomHeightfieldDataDestroy` | `void dGeomHeightfieldDataDestroy (dHeightfieldDataID d)` | include/ode/collision.h:1174 | Deallocate a `dHeightfieldDataID` (call after the geom using it is removed). |
| `dGeomHeightfieldDataBuildCallback` | `void dGeomHeightfieldDataBuildCallback (dHeightfieldDataID d, void* pUserData, dHeightfieldGetHeight* pCallback, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1218 | Configure heightfield to compute heights via a user callback per `(x,z)` sample. |
| `dGeomHeightfieldDataBuildByte` | `void dGeomHeightfieldDataBuildByte (dHeightfieldDataID d, const unsigned char* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1266 | Configure from an 8-bit `unsigned char` array; if `bCopyHeightData==0` the array must outlive the heightfield. |
| `dGeomHeightfieldDataBuildShort` | `void dGeomHeightfieldDataBuildShort (dHeightfieldDataID d, const short* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1314 | Configure from a 16-bit `short` array. |
| `dGeomHeightfieldDataBuildSingle` | `void dGeomHeightfieldDataBuildSingle (dHeightfieldDataID d, const float* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1364 | Configure from a single-precision `float` array. (The single-precision builder is `...BuildSingle`, not `...BuildFloat`.) |
| `dGeomHeightfieldDataBuildDouble` | `void dGeomHeightfieldDataBuildDouble (dHeightfieldDataID d, const double* pHeightData, int bCopyHeightData, dReal width, dReal depth, int widthSamples, int depthSamples, dReal scale, dReal offset, dReal thickness, int bWrap)` | include/ode/collision.h:1414 | Configure from a double-precision `double` array. |
| `dGeomHeightfieldDataSetBounds` | `void dGeomHeightfieldDataSetBounds (dHeightfieldDataID d, dReal minHeight, dReal maxHeight)` | include/ode/collision.h:1436 | Manually set min/max height for the AABB; must be set before binding to a geom (the AABB is not recomputed after first generation). |

### Trimesh

The trimesh creator(s) live in **include/ode/collision_trimesh.h**, not collision.h. The class id is `dTriMeshClass`. See references/components/geom-trimesh.md. (`dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback*, dTriArrayCallback*, dTriRayCallback*)` is at include/ode/collision_trimesh.h:253 — grep-confirmed.)

### Placeable-geom binding and pose

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomSetBody` | `void dGeomSetBody (dGeomID geom, dBodyID body)` | include/ode/collision.h:105 | Bind a placeable geom to a body so they share position/rotation; `body=0` detaches and gives the geom its own pose. |
| `dGeomGetBody` | `dBodyID dGeomGetBody (dGeomID geom)` | include/ode/collision.h:114 | Body bound to a geom (`0` if none / static); used in the near-callback to attach contact joints. |
| `dGeomSetPosition` | `void dGeomSetPosition (dGeomID geom, dReal x, dReal y, dReal z)` | include/ode/collision.h:131 | Set a placeable geom's position (also moves its body if attached); debug-build error on non-placeable geoms. |
| `dGeomSetRotation` | `void dGeomSetRotation (dGeomID geom, const dMatrix3 R)` | include/ode/collision.h:146 | Set a placeable geom's rotation matrix (also rotates its body if attached). |
| `dGeomSetOffsetPosition` | `void dGeomSetOffsetPosition (dGeomID geom, dReal x, dReal y, dReal z)` | include/ode/collision.h:542 | Local positional offset from the geom's body (geom must be attached; offset auto-created). Modern replacement for geom transforms. |
| `dGeomSetOffsetRotation` | `void dGeomSetOffsetRotation (dGeomID geom, const dMatrix3 R)` | include/ode/collision.h:558 | Local rotational offset from the geom's body. |
| `dGeomDestroy` | `void dGeomDestroy (dGeomID geom)` | include/ode/collision.h:65 | Destroy any geom (one function for all types); removes it from its space first. |

### Geom introspection, user data, AABB

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomGetClass` | `int dGeomGetClass (dGeomID geom)` | include/ode/collision.h:307 | Geom class id (`dSphereClass`, `dBoxClass`, `dPlaneClass`, …). |
| `dGeomGetSpace` | `dSpaceID dGeomGetSpace (dGeomID)` | include/ode/collision.h:280 | The space containing a geom, or NULL. |
| `dGeomIsSpace` | `int dGeomIsSpace (dGeomID geom)` | include/ode/collision.h:270 | Non-zero if the geom is actually a space (spaces are geoms). |
| `dGeomGetAABB` | `void dGeomGetAABB (dGeomID geom, dReal aabb[6])` | include/ode/collision.h:261 | Axis-aligned bounding box as `(minx,maxx,miny,maxy,minz,maxz)`. |
| `dGeomSetData` | `void dGeomSetData (dGeomID geom, void* data)` | include/ode/collision.h:75 | Store a user data pointer on the geom (commonly used in the near-callback to identify geoms). |
| `dGeomGetData` | `void *dGeomGetData (dGeomID geom)` | include/ode/collision.h:84 | Retrieve the user data pointer stored on the geom. |

### Category / collide bitfields and enable

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomSetCategoryBits` | `void dGeomSetCategoryBits (dGeomID geom, unsigned long bits)` | include/ode/collision.h:322 | Set the geom's category bitfield (>=32 bits, all set by default). |
| `dGeomSetCollideBits` | `void dGeomSetCollideBits (dGeomID geom, unsigned long bits)` | include/ode/collision.h:337 | Set the geom's collide bitfield (>=32 bits, all set by default); gates which categories this geom is tested against. |
| `dGeomEnable` | `void dGeomEnable (dGeomID geom)` | include/ode/collision.h:374 | Enable a geom (default state). |
| `dGeomDisable` | `void dGeomDisable (dGeomID geom)` | include/ode/collision.h:389 | Disable a geom so `dSpaceCollide`/`dSpaceCollide2` ignore it while it stays a space member. |
| `dGeomIsEnabled` | `int dGeomIsEnabled (dGeomID geom)` | include/ode/collision.h:405 | Non-zero if the geom is enabled. |

### Broadphase spaces (a space IS a geom)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dSimpleSpaceCreate` | `dSpaceID dSimpleSpaceCreate (dSpaceID space)` | include/ode/collision_space.h:52 | Simple brute-force O(n²) space; arg is an optional parent space to insert into (`0` for none). |
| `dHashSpaceCreate` | `dSpaceID dHashSpaceCreate (dSpaceID space)` | include/ode/collision_space.h:53 | Multi-resolution hash-table space (general-purpose default); arg is optional parent. |
| `dQuadTreeSpaceCreate` | `dSpaceID dQuadTreeSpaceCreate (dSpaceID space, const dVector3 Center, const dVector3 Extents, int Depth)` | include/ode/collision_space.h:54 | Quadtree over a fixed region (`Center`/`Extents`) with subdivision `Depth`; good for largely planar static worlds. |
| `dSweepAndPruneSpaceCreate` | `dSpaceID dSweepAndPruneSpaceCreate (dSpaceID space, int axisorder)` | include/ode/collision_space.h:66 | Sweep-and-prune space; `axisorder` is one of `dSAP_AXES_*` (XZY/ZXY usually best with Y up). |
| `dSpaceDestroy` | `void dSpaceDestroy (dSpaceID)` | include/ode/collision_space.h:70 | Destroy a space; with cleanup mode 1 (default) all contained geoms are destroyed too. |
| `dSpaceSetCleanup` | `void dSpaceSetCleanup (dSpaceID space, int mode)` | include/ode/collision_space.h:75 | Whether destroying the space destroys its geoms (`1`=default cleanup, `0`=leave geoms). |
| `dSpaceSetSublevel` | `void dSpaceSetSublevel (dSpaceID space, int sublevel)` | include/ode/collision_space.h:100 | Set a space's sublevel, controlling recursion in `dSpaceCollide2`; **not** auto-updated on nesting. |
| `dHashSpaceSetLevels` | `void dHashSpaceSetLevels (dSpaceID space, int minlevel, int maxlevel)` | include/ode/collision_space.h:72 | Set the min/max cell-size levels (powers of two) for a hash space. |
| `dSpaceAdd` | `void dSpaceAdd (dSpaceID, dGeomID)` | include/ode/collision_space.h:149 | Add a geom to a space (usually done implicitly by passing the space to a `dCreate*` call). |
| `dSpaceRemove` | `void dSpaceRemove (dSpaceID, dGeomID)` | include/ode/collision_space.h:150 | Remove a geom from a space without destroying it. |
| `dSpaceQuery` | `int dSpaceQuery (dSpaceID, dGeomID)` | include/ode/collision_space.h:151 | Non-zero if the given geom is contained in the space. |
| `dSpaceClean` | `void dSpaceClean (dSpaceID)` | include/ode/collision_space.h:152 | Bring the space's internal acceleration structures up to date now; rarely needed directly — `dSpaceCollide` cleans automatically. |
| `dSpaceGetNumGeoms` | `int dSpaceGetNumGeoms (dSpaceID)` | include/ode/collision_space.h:153 | Number of geoms in the space. |
| `dSpaceGetGeom` | `dGeomID dSpaceGetGeom (dSpaceID, int i)` | include/ode/collision_space.h:154 | The i-th geom in the space. |
| `dSpaceGetClass` | `int dSpaceGetClass (dSpaceID space)` | include/ode/collision_space.h:176 | Class id of a space (`dSimpleSpaceClass`, `dHashSpaceClass`, …). |

The broadphase iterators `dSpaceCollide` / `dSpaceCollide2`, the `dNearCallback` typedef, `dCollide`, and the per-pair contact build are documented in references/collision-and-contacts.md.

### Types (geom/space side)

| Symbol | Summary | Source |
| --- | --- | --- |
| `dHeightfieldDataID` | `typedef struct dxHeightfieldData*` — opaque handle for heightfield sample data, created/destroyed and configured separately from the geom. | include/ode/collision.h:1104 |
| `dHeightfieldGetHeight` | `typedef dReal dHeightfieldGetHeight (void* p_user_data, int x, int z)` — callback returning a sample height for the callback-based heightfield. | include/ode/collision.h:1124 |
| `dGeomClass` | `struct {int bytes; dGetColliderFnFn *collider; dGetAABBFn *aabb; dAABBTestFn *aabb_test; dGeomDtorFn *dtor;}` — used with `dCreateGeomClass` to register a custom geom type. | include/ode/collision.h:1497 |

`dContactGeom`, `dContact`, `dSurfaceParameters`, and `dNearCallback` are the contact-pipeline types — see references/collision-and-contacts.md.

## Key rules

- **A geom carries no mass and a body carries no shape; bind them with `dGeomSetBody`** so their pose is shared, and pass `body=0` to detach (the geom then owns its own pose) (include/ode/collision.h:88).
- **`dCreateBox` takes FULL side lengths `lx,ly,lz`, not half-extents** — a 2×2×2 cube is `dCreateBox(space,2,2,2)`, with the reference point at its center (include/ode/collision.h:1001).
- **Planes are non-placeable: define them by the equation `a*x+b*y+c*z=d`, never by `dGeomSetPosition`.** A horizontal floor at height 0 is `dCreatePlane(space,0,0,1,0)`; calling a position/rotation setter on a plane (or fixed heightfield) is a debug-build runtime error (include/ode/collision.h:88).
- **Static geoms have `dGeomGetBody == 0`.** Leave the ground plane unbound; the contact pipeline reads body 0 as the immovable environment (include/ode/collision.h:114).
- **Position/rotation setters require a placeable geom and also move the attached body**; `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation` offset the geom *from* its body and require the geom to be attached (include/ode/collision.h:131).
- **Destroying a space with cleanup mode 1 (the default) destroys every geom inside it**, so you usually don't `dGeomDestroy` each geom yourself; `dGeomDestroy` works for any geom type and removes it from its space first (include/ode/collision.h:59).
- **Spaces are geoms: `dGeomIsSpace` distinguishes them, and `dSpaceCollide` does NOT recurse into nested sub-spaces** — a sub-space may arrive as `o1`/`o2` and you must call `dSpaceCollide2` (or `dCollide`) on it yourself to descend (include/ode/collision.h:807).
- **Disabled geoms and geoms whose category/collide bits don't intersect are skipped by the broadphase; new geoms default to enabled with all category and collide bits set** (include/ode/collision.h:365).
- **Category/collide filtering is directional-pair logic**: a candidate pair is kept only when one geom's category bits intersect the other's collide bits — set both sides to make two groups ignore each other (include/ode/collision.h:322).
- **A heightfield has a two-object lifecycle**: `dGeomHeightfieldDataCreate` → one `dGeomHeightfieldDataBuild{Callback,Byte,Short,Single,Double}` → `dCreateHeightfield(space,data,bPlaceable)`; destroy the data with `dGeomHeightfieldDataDestroy` *after* the geom, and for non-copied / callback data the source buffer must outlive the geom (include/ode/collision.h:1147).
- **`dGeomTransform` (`dCreateGeomTransform` and its `dGeomTransformSet/Get*`) is DEPRECATED** (`ODE_API_DEPRECATED`); use `dGeomSetOffsetPosition`/`dGeomSetOffsetRotation` to offset a geom from its body instead (include/ode/collision.h:1089).
- **`dCreateCCylinder` is not a distinct function — it is a `#define` alias for `dCreateCapsule`** (and `dCCylinderClass` aliases `dCapsuleClass`); a capsule's `length` covers only the cylinder section, with hemispherical caps of `radius` added at each end (include/ode/collision.h:1056).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `dCreateBox(space, 0.5, 0.5, 0.5)` for a half-metre cube | `dCreateBox(space, 0.5, 0.5, 0.5)` *only if you want a 0.5 m box*; for a 1 m cube use `dCreateBox(space, 1, 1, 1)` | `dCreateBox` arguments are FULL side lengths, not half-extents (include/ode/collision.h:1001). |
| `dGeomID p = dCreatePlane(space,0,0,1,0); dGeomSetPosition(p,0,0,5);` | `dGeomID p = dCreatePlane(space,0,0,1,5);` | Planes are non-placeable; the `a,b,c,d` equation positions them, and `dGeomSetPosition` on a plane is a debug-build runtime error (include/ode/collision.h:88). |
| Binding a geom to its body but leaving the ground plane attached to a body too | Leave the ground plane with `body=0`; only bind dynamic geoms with `dGeomSetBody` | A static geom must have `dGeomGetBody == 0` so the contact pipeline treats it as the immovable environment (include/ode/collision.h:114). |
| `dGeomDestroy` each geom **and** `dSpaceDestroy(space)` (double free) | `dSpaceDestroy(space)` alone (cleanup mode 1) frees the contained geoms | Destroying a space with default cleanup destroys all its geoms automatically (include/ode/collision.h:59). |
| Expecting `dSpaceCollide` to recurse into a sub-space you added | In the callback, `if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) dSpaceCollide2(o1,o2,data,cb);` | `dSpaceCollide` does not descend into nested spaces; you must call `dSpaceCollide2`/`dCollide` yourself (include/ode/collision.h:807). |
| `dCreateCCylinder(space,r,l)` assumed to be a different shape from a capsule | `dCreateCapsule(space,r,l)` (or the `dCreateCCylinder` alias) — same thing | `dCreateCCylinder` is a `#define` alias for `dCreateCapsule` (include/ode/collision.h:1056). |

## Named constants

### Geom class ids — returned by `dGeomGetClass` (enum opens at include/ode/collision.h:877)

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dSphereClass` | 0 | include/ode/collision.h:877 | Sphere geom class id (enum starts here, others follow in order). |
| `dBoxClass` | 1 | include/ode/collision.h:877 | Box geom class id. |
| `dCapsuleClass` | 2 | include/ode/collision.h:877 | Capsule geom class id (`dCCylinderClass` is an alias). |
| `dCylinderClass` | 3 | include/ode/collision.h:877 | Cylinder geom class id. |
| `dPlaneClass` | 4 | include/ode/collision.h:877 | Plane geom class id (non-placeable). |
| `dRayClass` | 5 | include/ode/collision.h:877 | Ray geom class id. |
| `dConvexClass` | 6 | include/ode/collision.h:877 | Convex geom class id. |
| `dGeomTransformClass` | 7 | include/ode/collision.h:877 | Geom-transform class id (deprecated wrapper). |
| `dTriMeshClass` | 8 | include/ode/collision.h:877 | Trimesh geom class id (creator lives in collision_trimesh.h). |
| `dHeightfieldClass` | 9 | include/ode/collision.h:877 | Heightfield geom class id. |

### Space class ids — between `dFirstSpaceClass` and `dLastSpaceClass` (include/ode/collision.h:889)

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dSimpleSpaceClass` | `dFirstSpaceClass` | include/ode/collision.h:889 | Class id of a simple space. |
| `dHashSpaceClass` | next | include/ode/collision.h:889 | Class id of a hash space. |
| `dSweepAndPruneSpaceClass` | next | include/ode/collision.h:889 | Class id of a sweep-and-prune space. |
| `dQuadTreeSpaceClass` | `dLastSpaceClass` | include/ode/collision.h:889 | Class id of a quadtree space. |

### Sweep-and-prune axis orders — for `dSweepAndPruneSpaceCreate`

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dSAP_AXES_XYZ` | `((0)\|(1<<2)\|(2<<4))` | include/ode/collision_space.h:59 | X-major SAP axis order. |
| `dSAP_AXES_XZY` | `((0)\|(2<<2)\|(1<<4))` | include/ode/collision_space.h:60 | Recommended when Y is up. Full set: XYZ, XZY, YXZ, YZX, ZXY, ZYX. |
| `dSAP_AXES_YXZ` | `((1)\|(0<<2)\|(2<<4))` | include/ode/collision_space.h:61 | Y-major SAP axis order. |
| `dSAP_AXES_YZX` | `((1)\|(2<<2)\|(0<<4))` | include/ode/collision_space.h:62 | SAP axis order. |
| `dSAP_AXES_ZXY` | `((2)\|(0<<2)\|(1<<4))` | include/ode/collision_space.h:63 | Also good when Y is up. |
| `dSAP_AXES_ZYX` | `((2)\|(1<<2)\|(0<<4))` | include/ode/collision_space.h:64 | SAP axis order. |

The surface-mode bit flags (`dContactMu2`, `dContactBounce`, `dContactSoftERP`/`dContactSoftCFM`, …) and the `dCollide` flag `CONTACTS_UNIMPORTANT` belong to the contact pipeline — see references/collision-and-contacts.md.

## Things to never invent

(Contact-flag/contact-pipeline invented-name traps — `dContactSoft`, `dContactFriction`/`dContactFriction2`, `dContactGroupEmpty` — live in references/collision-and-contacts.md.)

- **`dCreateCCylinder` as a separate function** — it is a `#define` alias for `dCreateCapsule` (include/ode/collision.h:1056).
- **`dSpaceStep` / `dSpaceUpdate`** — no such functions; spaces are not stepped.
- **`dGeomCollide`** — the function is `dCollide`, not `dGeomCollide`.
- **`dCreateTriMesh` in collision.h** — the trimesh creator lives in collision_trimesh.h as `dCreateTriMesh*` variants, not in collision.h; the class is `dTriMeshClass`.
- **`dGeomSetCollisionBits`** — it is `dGeomSetCollideBits`.
- **`dSpaceCollideAll` / `dWorldCollide`** — no such API.
- **`dGeomHeightfieldDataBuildFloat`** — the single-precision builder is `dGeomHeightfieldDataBuildSingle`; there is also `dGeomHeightfieldDataBuildDouble`.

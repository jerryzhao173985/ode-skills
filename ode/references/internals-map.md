# ODE Internals Map (ode/src — below the public headers)

> Source of truth: ode/src/*.cpp and ode/src/joints/*.cpp (the implementation), with the public contract in include/ode/*.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- This map is **internals** — everything under `ode/src/` is OUTSIDE the public `include/ode` surface and is NOT part of the supported API. Use it only to debug ODE behavior below the documented headers; cite the header for any API contract (ode/src/ode.cpp:1828).
- The public entry points (e.g. `dWorldStep`, `dCollide`) are thin wrappers in `ode/src/` that dispatch into the real machinery: steppers, the island partitioner, and the collider dispatch table.
- Two distinct solvers back stepping: the **big-matrix exact** path (`step.cpp` + `lcp.cpp`, behind `dWorldStep`) and the **iterative SOR/PGS** path (`quickstep.cpp`, behind `dWorldQuickStep`). They share only island-splitting (`util.cpp`) and per-body integration (`dxStepBody`).
- All inter-geom collision is routed through a static `colliders[class1][class2]` function-pointer table in `collision_kernel.cpp` — never by direct calls (ode/src/collision_kernel.cpp:141).
- The in-repo Doxygen docs are an **API-reference stub**: `main.dox` is 22 lines and "not yet complete"; the real reference is the `@defgroup` module tree embedded in the headers, and the Doxygen INPUT is fixed to `include/ode` only (ode/doc/main.dox:15-19, ode/doc/Doxyfile.in:746).

## When to use

- You are debugging *why* a step, contact, joint, or mass result is wrong and need to know which `.cpp`/`.h` implements it.
- You need to map a public header symbol to its implementation file to set a breakpoint or read the numerics.
- You are deciding between `dWorldStep` and `dWorldQuickStep` and want to know which solver file owns accuracy vs. speed.
- You want to know where the in-repo docs live and what they cover vs. defer to the wiki.
- Do NOT use this map to cite an `ode/src` symbol as public API — for that, see the header references.

## Public API (C)

These are the public symbols whose **implementation** lives in `ode/src`. The Signature is the implementation definition; the Source is the implementation `file:line`. The API *contract* belongs to the header noted in Purpose.

### Step / world pipeline (backs include/ode/objects.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dWorldStep` | `int dWorldStep (dWorldID w, dReal stepsize)` | ode/src/ode.cpp:1828 | Public entry for the big-matrix exact stepper; wrapper that runs `dxProcessIslands` with `dxStepIsland`. |
| `dWorldQuickStep` | `int dWorldQuickStep (dWorldID w, dReal stepsize)` | ode/src/ode.cpp:1847 | Public entry for the iterative SOR/PGS QuickStep solver; wrapper that runs `dxProcessIslands` with `dxQuickStepIsland`. |
| `dBodyCreate` | `dxBody *dBodyCreate (dxWorld *w)` | ode/src/ode.cpp:240 | Allocates and links a rigid body into the world; data-structure code, not numerics. |

### Collision pipeline (backs include/ode/collision.h, collision_space.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCollide` | `int dCollide (dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)` | ode/src/collision_kernel.cpp:292 | Central collision dispatcher: looks up `colliders[class1][class2]` and invokes the right primitive collider (reversing args if needed). |
| `dSpaceCollide` | `void dSpaceCollide (dxSpace *space, void *data, dNearCallback *callback)` | ode/src/collision_space.cpp:779 | Broadphase driver: enumerates candidate geom pairs and calls the user near-callback. |
| `dCreateHeightfield` | `dGeomID dCreateHeightfield( dSpaceID space, dHeightfieldDataID data, int bPlaceable )` | ode/src/heightfield.cpp:886 | Terrain geom factory; tessellates the height grid against other geoms. |
| `dSweepAndPruneSpaceCreate` | `dSpaceID dSweepAndPruneSpaceCreate( dxSpace* space, int axisorder )` | ode/src/collision_sapspace.cpp:213 | Sweep-and-Prune broadphase space (short alias `dSAPSpaceCreate` — verify before using). |
| `dCreateTriMesh` | `dGeomID dCreateTriMesh(dSpaceID space, ...)` | ode/src/collision_trimesh_gimpact.cpp:393 | Trimesh geom factory; **same symbol resolves to a different backend file per build config** (see Key rules). |

### Library lifecycle (backs include/ode/odeinit.h)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dInitODE2` | `int dInitODE2(unsigned int uiInitFlags/*=0*/)` | ode/src/odeinit.cpp:516 | Ref-counted library init; sets up OU customizations, TLS, trimesh subsystem. |
| `dCloseODE` | `void dCloseODE()` | ode/src/odeinit.cpp:569 | Library finalization; releases the resources `dInitODE2` allocated. |

### Internal machinery (NOT public API — reached only through the above)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dxStepIsland` | `void dxStepIsland(const dxStepperProcessingCallContext *callContext)` | ode/src/step.cpp:513 | Big-matrix integrator for one island: builds global Jacobian + mass matrix, solves via LCP, integrates. Staged `dxStepIsland_Stage0..4` split work for threading (ode/src/step.cpp:576). |
| `dxQuickStepIsland_Stage0_Bodies` | `static void dxQuickStepIsland_Stage0_Bodies(dxQuickStepperStage0BodiesCallContext *callContext)` | ode/src/quickstep.cpp:573 | First of the multi-stage threaded QuickStep solver; SOR iteration over constraint rows, split across the threading pool. |
| `dxProcessIslands` | `bool dxProcessIslands (dxWorld *world, const dxWorldProcessIslandsInfo &islandsInfo, ...)` | ode/src/util.cpp:886 | Island partitioner: splits bodies+joints into connected components and runs the chosen stepper per island. (Inventory listed util.cpp:731; verified definition is util.cpp:886 in this checkout — see [VERIFY].) |
| `dInternalHandleAutoDisabling` | `void dInternalHandleAutoDisabling (dxWorld *world, dReal stepsize)` | ode/src/util.cpp:427 | Per-step auto-disable: averages lin/ang velocity over a sample window and disables idle bodies. Backs `dWorldSetAutoDisable*`. |
| `dxStepBody` | `void dxStepBody (dxBody *b, dReal h)` | ode/src/util.cpp:583 | Integrates one body's position/orientation from its velocity over timestep `h`; shared by both steppers. |
| `setCollider` | `static void setCollider (int i, int j, dColliderFn *fn)` | ode/src/collision_kernel.cpp:148 | Registers one collider function into the `colliders[i][j]` dispatch table at init. |
| `dLCP::dLCP` | `dLCP::dLCP (unsigned n, unsigned nskip, unsigned nub, dReal *Adata, ...)` | ode/src/lcp.cpp:431 | Boxed-LCP solver (Dantzig/Baraff-style index-set pivoting) used by the big-matrix `dWorldStep`. |

## Source-file navigation map

The core deliverable: which `ode/src` file backs which public header and what it implements. **All of these are internals — outside `include/ode`.**

| Source file | Backs (public header) | What it implements |
| --- | --- | --- |
| `ode/src/ode.cpp` | include/ode/objects.h | World+body data structures: `dWorldCreate/Destroy`, all `dWorldSet/Get*` accessors (ERP, CFM, gravity, damping, auto-disable, quickstep iters), `dBodyCreate/Destroy` (ode.cpp:240), and the `dWorldStep`/`dWorldQuickStep` wrappers (ode.cpp:1828/1847). "Mostly concerned with data structures, not numerics." |
| `ode/src/objects.cpp` | objects.h / threading_impl.h | Base `dObject`/`dxWorld` construction + default threading wiring; shared object lifecycle used by ode.cpp. |
| `ode/src/odeinit.cpp` | include/ode/odeinit.h | Library init/shutdown: `dInitODE2` (odeinit.cpp:516), `dCloseODE` (569), `dAllocateODEDataForThread`. Ref-counted; sets up OU, TLS, trimesh subsystem. |
| `ode/src/step.cpp` | objects.h (`dWorldStep` numerics) | Big-matrix exact stepper. `dxStepIsland` (step.cpp:513) builds the full constraint Jacobian + inverse-mass matrix, solves the mixed LCP via lcp.cpp, integrates. `dxStepIsland_Stage0..4` thread the work; `dxEstimateStepMemoryRequirements` sizes scratch. |
| `ode/src/quickstep.cpp` | objects.h (`dWorldQuickStep` numerics) | Iterative QuickStep (projected Gauss-Seidel / SOR) stepper — largest src file. Heavily staged (`dxQuickStepIsland_Stage0_Bodies/_Joints/_Stage1/_Stage2a/2b/_Stage3`, quickstep.cpp:573+) for multithreaded constraint-row relaxation; `dxEstimateQuickStepMemoryRequirements` sizes scratch. |
| `ode/src/util.cpp` | objects.h (step-memory + threading APIs) | World-process plumbing shared by both steppers: `dxProcessIslands` (util.cpp:886) partitions into islands and runs the chosen stepper; `dInternalHandleAutoDisabling` (427) velocity-threshold sleep; `dxStepBody` (583) integrates one body; `dxWorldProcessContext`/`MemArena` provide per-step scratch arenas; `dxReallocateWorldProcessContext` manages that memory. |
| `ode/src/lcp.cpp` / `lcp.h` | (internal — feeds step.cpp) | Boxed Linear Complementarity Problem solver for the big-matrix step. `dLCP` class (lcp.cpp:431) with index-set pivoting (`transfer_i_to_C`/`_from_N_to_C`/`_from_C_to_N`, `solve1`, `unpermute_X`). Core of exact-step stability. |
| `ode/src/mass.cpp` | include/ode/mass.h | Mass/inertia tensor construction: `dMassSetSphere/Box/Capsule/Cylinder/Trimesh`, `dMassAdjust`, `dMassTranslate`, `dMassRotate`, `dMassAdd`. Uses collision_kernel + (optionally) trimesh internals for trimesh mass. |
| `ode/src/joints/joint.cpp` | objects.h (`dJoint*` generic API) | Base `dxJoint` class: constructor/destructor, `setRelativeValues`, `isEnabled`, the shared `getInfo1`/`getInfo2` constraint-row helpers and `GI2_*` row-index constants. Common machinery every constraint subclass extends. |
| `ode/src/joints/contact.cpp` | objects.h + contact.h | Contact joint generated per collision contact point: `getInfo1` (contact.cpp:49) / `getInfo2` (126) build the friction-cone constraint rows from a `dContact` (mu, bounce, soft_cfm/erp). `dJointCreateContact`. The bridge between collision and dynamics. |
| `ode/src/joints/{ball,hinge,hinge2,slider,universal,fixed,null,amotor,lmotor,pr,pu,piston,plane2d,dball,dhinge,transmission}.cpp` | objects.h (`dJointCreate*`) | One file per joint type, each with its own `getInfo1/2` + `dJointCreate<Type>` and setters. `joint_internal.h`/`joints.h`/`joint.h` are the internal headers (limot/`dxJointLimitMotor` helpers, `GI2_*` row indices). |
| `ode/src/collision_kernel.cpp` | include/ode/collision.h | Collision dispatch core. `dCollide` (kernel.cpp:292) routes through the static `colliders[][]` table (141); `setCollider`/`setAllColliders` (148) register per-class-pair functions; `dCreateGeomClass` registers user geom classes; `dGeomDestroy`/`dGeomSetBody`/`dGeomMoved`/`recomputeAABB` manage geom lifecycle + dirty-AABB tracking. |
| `ode/src/collision_space.cpp` | include/ode/collision_space.h | Base space + built-in spaces: `dSimpleSpaceCreate` (O(n²) brute force) and `dHashSpaceCreate` (spatial hash). `dSpaceCollide` (space.cpp:779) / `dSpaceCollide2` drive broadphase pair generation; `dSpaceAdd/Destroy` manage membership. |
| `ode/src/collision_quadtreespace.cpp` | collision_space.h | Quadtree broadphase: `dQuadTreeSpaceCreate` (center/extents/depth). Good for large, roughly-2D static layouts. |
| `ode/src/collision_sapspace.cpp` | collision_space.h | Sweep-and-Prune broadphase: `dSweepAndPruneSpaceCreate` (sapspace.cpp:213, also exposed as `dSAPSpaceCreate`). Sorts AABB endpoints along axes for incremental pruning. |
| `ode/src/collision_std.h` | (internal header for collision.h primitives) | Declares all built-in primitive colliders with the `dColliderFn` signature (`dCollideSphereSphere`/`SphereBox` (std.h:42)/`SpherePlane`/`BoxBox` (46)/`BoxPlane`/`CapsuleSphere`/...). These names are what `setAllColliders` wires into the dispatch table. |
| `ode/src/collision_transform.cpp` | collision.h (GeomTransform) | Geom-transform wrapper: `dCreateGeomTransform` wraps an encapsulated geom with local pos/rotation; `dCollideTransform` recomputes the child's world transform then collides; `dGeomTransformGetGeom`. |
| `ode/src/collision_util.cpp` | (internal — collision_util.h) | Shared geometry helpers for primitive colliders: `dCollideSpheres`, `dLineClosestApproach`, `dClosestLineSegmentPoints`, `dClosestLineBoxPoints`, `dBoxTouchesBox`, polygon clip helpers. Pure math, no public API. |
| `ode/src/{box,sphere,capsule,cylinder,plane,ray,convex}.cpp` | collision.h (primitive geoms) | Per-primitive factories + their collider impls: `dCreateBox` (box.cpp:80), `dCreateSphere` (sphere.cpp:70), `dCreateCapsule` (capsule.cpp:77), `dCreateCylinder` (cylinder.cpp:84), `dCreatePlane` (plane.cpp:110), `dCreateRay` (ray.cpp:92), `dCreateConvex` (convex.cpp:208). Each also implements its `dCollideXY` routines from collision_std.h. This is where the actual sphere/box/etc. geometry math lives — NOT in dCollide. |
| `ode/src/collision_cylinder_{box,plane,sphere,trimesh}.cpp`, `collision_convex_trimesh.cpp`, `collision_libccd.cpp` | collision.h (specialized pairs) | Pairwise colliders that don't fit the generic primitive files (cylinder-vs-X, convex-vs-trimesh), plus the libccd (GJK/MPR) fallback for convex shapes where analytic routines are absent. Registered into `colliders[][]`. Backend internals — survey only. |
| `ode/src/heightfield.cpp` | collision.h (heightfield API) | Heightfield geom + data: `dGeomHeightfieldDataCreate`, Build variants Callback/Byte/Short/Single/Double, `dCreateHeightfield` (heightfield.cpp:886). Terrain collision by tessellating the grid. |
| `ode/src/collision_trimesh_internal.cpp` / `.h` | (internal — shared by both trimesh backends) | Backend-agnostic trimesh glue: shared `dxTriMesh`/`dxTriMeshData` base classes consumed by both opcode and gimpact. Survey only. |
| `ode/src/collision_trimesh_{opcode,gimpact,disabled}.cpp` | collision.h (trimesh API) | Three **mutually-exclusive build-time backends**. opcode and gimpact each define `dGeomTriMeshDataCreate` + `dCreateTriMesh` (gimpact.cpp:393); disabled provides stubs when trimesh is off. The same symbol resolves to a different file per build config. |
| `ode/src/collision_trimesh_{box,sphere,ccylinder,plane,ray,trimesh,trimesh_old}.cpp` | collision.h (trimesh pairs) | Per-pair trimesh colliders (trimesh vs box/sphere/capsule/plane/ray/trimesh). `trimesh_trimesh_old.cpp` is the legacy impl kept for reference. Backend internals — survey only. |
| `ode/src/matrix.cpp` / `matrix.h` | (internal numerics) | Dense linear-algebra primitives (factor/solve, multiply, transpose) over `dReal` arrays, plus threaded LDLT glue (threaded_solver_ldlt.h). Used by step/quickstep/lcp. |
| `ode/src/fast{ldltfactor,ldltsolve,lsolve,ltsolve,dot,vecscale}.cpp` (+ `*_impl.h`) | (internal numeric hot path) | Optimized BLAS-like kernels for the threaded LDLT solver: LDLT factorization, L and Lᵀ solves, dot product (`dxDot`, fastdot.cpp:33), vector scaling. The numeric hot path of Step/QuickStep. |
| `ode/src/threaded_solver_ldlt.h`, `threadingutils.h` | (internal) | Header-only orchestration of the multithreaded LDLT equation solve consumed by the fast* kernels and the steppers. |
| `ode/src/threading_{base,impl}.cpp`, `threading_pool_{posix,win}.cpp`, `default_threading.cpp` | include/ode/threading.h + threading_impl.h | Threading subsystem: base wrapper, self-threaded impl object, POSIX + Windows thread pools, and the default single-thread implementation. Survey only. |
| `ode/src/odeou.cpp`, `odetls.cpp`, `resource_control.cpp` | (internal init/runtime support) | OU library integration (memory/assertion customization, `COdeOu`), thread-local storage, resource-requirement bookkeeping used by `dInitODE2`/`dAllocateODEDataForThread`. |
| `ode/src/obstack.cpp` | (internal — obstack.h) | Legacy obstack (stack-of-arenas) allocator for transient per-step allocations; largely superseded by the `dxWorldProcessMemArena` path in util.cpp. |
| `ode/src/error.cpp` / `error.h` | include/ode/error.h | Diagnostic dispatch: `dSetErrorHandler`/`dSetDebugHandler`/`dSetMessageHandler` and `dError`/`dDebug`/`dMessage`. Default handlers print + (for errors) abort. **First place to look when ODE prints or aborts.** |
| `ode/src/memory.cpp` | include/ode/memory.h | Pluggable allocator front end: `dSetAllocHandler`/`dSetReallocHandler`/`dSetFreeHandler` and `dAlloc`/`dRealloc`/`dFree`. Override here to route ODE allocations. |
| `ode/src/timer.cpp` | include/ode/timer.h | High-resolution profiling: `dTimerStart`/`dTimerNow`/`dTimerEnd`, `dTimerReport`, `dTimerTicksPerSecond`, `dTimerResolution`. Platform timers. |
| `ode/src/odemath.cpp` / `odemath.h` | include/ode/odemath.h | Non-inline vector/matrix helpers beyond the header macros: `dSafeNormalize3/4`, `dNormalize3/4`, `dPlaneSpace`. Provides the non-inline fallbacks. |
| `ode/src/rotation.cpp` | include/ode/rotation.h | Rotation conversions: `dRFromAxisAndAngle`, `dRFromEulerAngles`, `dRFrom2Axes`, `dQtoR`, `dRtoQ`, `dQMultiply*`, `dQfromR`, quaternion normalization. Quaternion format (s,vx,vy,vz). |
| `ode/src/misc.cpp` | include/ode/misc.h | Miscellaneous utilities: PRNG (`dRandReal`/`dRand`/`dRandSetSeed`), `dMaxDifference`, matrix print/test helpers. |
| `ode/src/export-dif.cpp` | include/ode/export-dif.h | `dWorldExportDIF`: serializes the entire world (bodies, joints, attached geoms) to `.dif` Debug Interchange Format text. **The go-to tool for capturing a broken world state for repro.** TODO notes list unexported spaces/geoms. |
| `ode/src/array.cpp` / `mat.cpp` / `nextafterf.c` | (internal) | `dArray` dynamic-array support (array.cpp), test/debug `dMatrix` wrapper (mat.cpp), and a portable `nextafterf()` shim (nextafterf.c). |
| `ode/src/{common,config,typedefs,objects,collision_kernel,collision_space_internal}.h` | (internal struct definitions) | Private structs (`dxWorld`, `dxBody`, `dxGeom`, `dxSpace`, `dxJoint`) and build config that the public opaque IDs (`dWorldID`, `dBodyID`, `dGeomID`) point to. **Read these to learn the real layout behind the opaque handles when debugging.** |
| `ode/src/*.cpp_deprecated`, `*.h_deprecated` (e.g. `stack.h`, `scrapbook.cpp`) | (dead code) | Retired code kept for history; not compiled. Ignore when debugging current behavior. |

## In-repo Doxygen docs (what they cover, what they defer)

- `ode/doc/main.dox` is a **22-line stub** (`@mainpage`): copyright, the one-line project description, "This document describes the library API", a "this document is not yet complete!" disclaimer, and two links to the external Online Handbook (wiki). It carries **no API symbols** — do not treat it as a tutorial or a complete manual (ode/doc/main.dox:11-19).
- The **real reference** is the `@defgroup` module tree embedded in the public headers, assembled by Doxygen — NOT written in `main.dox`. The complete group set: `world` (objects.h:36), `bodies` (objects.h:870), `joints` (objects.h:1619), `disable` (objects.h:634), `damping` (objects.h:752), `collide` (collision.h:35) with subgroups `collide_sphere`/`collide_box`, `init` (odeinit.h:40), `coop` (cooperative.h:36) with subgroup `matrix_coop`, and `drawstuff` (drawstuff.h:23, outside the configured INPUT). A `threading` group is referenced by `@ingroup` tags but has no explicit `@defgroup` (Doxygen auto-creates it).
- **INPUT is `include/ode` only** with `RECURSIVE = NO` (ode/doc/Doxyfile.in:746,813): the generated reference covers the public C headers and intentionally **excludes `ode/src` internals, the collision backends, drawstuff, bindings, and tests**. This entire internals map is therefore *not* in the in-repo HTML docs.
- For concepts/tutorials the docs defer to the wiki (Online Handbook); for per-symbol contracts read the header `@defgroup`/comment prose; for internals (this map) read `ode/src` directly.

## Key rules

- `ode/src` is OUT OF SCOPE for the public API skill — treat these files only as a debugging map; never cite an `ode/src` symbol as a public API. The public surface is `include/ode/*.h` (e.g. `dWorldStep` is documented in objects.h; ode.cpp:1828 is merely its implementation) (ode/src/ode.cpp:1828).
- `dWorldStep` (big-matrix, exact, expensive LCP) lives in step.cpp + lcp.cpp; `dWorldQuickStep` (iterative SOR, faster/looser) lives in quickstep.cpp. Both are dispatched per-island by `dxProcessIslands` in util.cpp — pick the right file when debugging step accuracy vs. speed (ode/src/util.cpp:886).
- All inter-geom collision goes through the `colliders[][]` function-pointer table, NOT direct calls. To debug a missing/wrong contact, check `setCollider` registration first; a 0 entry means `dCollide` returns 0 contacts (ode/src/collision_kernel.cpp:148).
- `dCollide` only dispatches — the actual sphere/box/capsule geometry math lives in the per-primitive files (box.cpp, sphere.cpp, ...) and the `dCollideXY` routines declared in collision_std.h, e.g. `dCollideSphereBox` (ode/src/collision_std.h:42).
- Trimesh collision has THREE backends selected at build time: opcode, gimpact, and disabled. `dCreateTriMesh`/`dGeomTriMeshDataCreate` are defined in the active backend file, so the same symbol resolves differently per build config — verify the active macro before asserting which file owns it (ode/src/collision_trimesh_gimpact.cpp:393).
- The contact joint is the bridge between collision and dynamics: its `getInfo2` builds the friction-cone rows from a `dContact`, which is why contacts only have effect once you create a contact joint and attach it (ode/src/joints/contact.cpp:126).
- Auto-disable / sleeping is implemented in `dInternalHandleAutoDisabling`, called every step before integration; debug stuck-asleep or never-sleep bodies there (ode/src/util.cpp:427).
- When ODE prints a message or aborts, the dispatch goes through error.cpp's `dError`/`dDebug`/`dMessage` handlers — set your own handler there to catch it (ode/src/error.cpp top; backs include/ode/error.h per README.md:24-33).
- The in-repo Doxygen reference is `include/ode` only (RECURSIVE = NO); it does not document `ode/src`, the backends, or drawstuff — read those files directly (ode/doc/Doxyfile.in:746).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| Citing `ode/src/ode.cpp:1828` as the *definition* of the `dWorldStep` API in skill docs. | Cite the public header `include/ode/objects.h` for the API contract; use ode.cpp:1828 only as the implementation location when debugging internals. | The skill scope is the public C API; `ode/src` is internal and explicitly out of scope. The Doxygen INPUT is `include/ode` only (ode/doc/Doxyfile.in:746). |
| Assuming `dWorldStep` and `dWorldQuickStep` share solver code. | Know they are distinct: step.cpp/lcp.cpp (direct LCP) vs quickstep.cpp (iterative SOR). Only island-splitting (`dxProcessIslands`) and per-body integration (`dxStepBody`) are shared. | They have different accuracy/stability/performance characteristics; their constraint solvers are separate (ode/src/lcp.cpp:431 vs ode/src/quickstep.cpp:573). |
| Looking for sphere-vs-box collision math inside `dCollide` in collision_kernel.cpp. | `dCollide` only dispatches; the primitive routines live in box.cpp/sphere.cpp/etc. and the `dCollideXY` declarations in collision_std.h. | collision_kernel.cpp is a router; the geometry math is in the per-primitive files (ode/src/collision_kernel.cpp:292). |
| Treating `ode/doc/main.dox` as a complete manual and answering concept questions from it. | Use the `@defgroup` prose in the headers for concepts and the wiki for tutorials; main.dox carries almost no API content. | main.dox scopes itself to API reference and declares itself incomplete (ode/doc/main.dox:15-19). |
| Asserting the in-repo Doxygen docs cover internals/backends/drawstuff. | The Doxyfile INPUT is `include/ode` with RECURSIVE=NO, so only the public headers are documented. | ode/doc/Doxyfile.in:746,813 fix INPUT to include/ode with no recursion. |
| Trusting an internal helper's exact line number across ODE versions. | Re-grep the specific checkout before citing an internal line; only header API contracts are stable. | Line numbers drift; e.g. the inventory's `dxProcessIslands` at util.cpp:731 is actually util.cpp:886 in this checkout. |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomNumClasses` | (count of registered geom classes) | ode/src/collision_kernel.cpp:141 | Both-axes dimension of the static `colliders[dGeomNumClasses][dGeomNumClasses]` dispatch table; equals the number of registered geom classes (built-in + user classes via `dCreateGeomClass`). |
| `GI2_*` row-index constants | (constraint-row offsets) | ode/src/joints/joint.cpp | Constraint-row index constants used by `getInfo2` across joint subclasses; defined in the internal joints headers (joint.h). Internal only. |

The other modules in this map carry no public named constants of their own; their tunables (ERP, CFM, dParam*, etc.) are documented constants of the public headers, not of `ode/src`.

## Things to never invent

- Do NOT claim `ode/src` symbols are part of the public/supported API — they are internal implementation; the public surface is `include/ode/*.h` only.
- Do NOT invent which trimesh backend (opcode vs gimpact vs disabled) is active for a given build — it is a compile-time choice; verify via config.h / build flags before asserting.
- Do NOT assert exact line numbers for internal helpers without re-grepping the specific checkout — line numbers drift across ODE versions.
- Do NOT claim `dWorldStep` and `dWorldQuickStep` share a solver path beyond island-splitting (util.cpp) and per-body integration (`dxStepBody`); their constraint solvers are separate (lcp.cpp vs quickstep.cpp).
- Do NOT state `dSolveLCP`/`dLCP` behavior details beyond "boxed LCP pivoting solver" without reading lcp.cpp at the relevant checkout — the numerical method is intricate.
- Do NOT invent additional `@defgroup` module names. The complete set is: world, bodies, joints, disable, damping, collide, collide_sphere, collide_box, init, coop, matrix_coop, drawstuff (plus an implicit auto-created `threading` group with no `@defgroup`).
- Do NOT state a concrete ODE version from the docs: ode/doc/Doxyfile.in:41 and include/ode/version.h.in:4 both hold the unexpanded `@ODE_VERSION@` token, not a literal number.
- Do NOT claim `main.dox` documents APIs, concepts, or a getting-started guide beyond its 22 lines; it explicitly defers to the wiki and declares itself incomplete (main.dox:15-19).
- Do NOT assert the in-repo Doxygen reference covers `ode/src`, collision backends, drawstuff, or bindings; INPUT is fixed to `include/ode` with RECURSIVE=NO (Doxyfile.in:746,813).

## [VERIFY] markers

- `dxProcessIslands` definition line: the inventory cited `ode/src/util.cpp:731`, but `grep`/`sed` on this checkout shows util.cpp:731 is `dInternalHandleAutoDisabling` and the real `dxProcessIslands` definition is **util.cpp:886** (signature `bool dxProcessIslands (dxWorld *world, const dxWorldProcessIslandsInfo &islandsInfo, ...)`). This map cites the verified 886. Confirm against your checkout — line numbers drift.

# Version awareness, deprecations & changelog

> Source of truth: configure.ac, include/ode/version.h.in, and the deprecation markers in include/ode/{collision.h,collision_trimesh.h,objects.h,mass.h,odeconfig.h} (cross-checked against CHANGELOG.txt). Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- This source tree is **ODE 0.16** (`configure.ac:2` `AC_INIT([ODE],[0.16])`; `configure.ac:3` `ODE_VERSION=0.16`). The latest *published* upstream release is **0.16.6** (2025-01-16, per Wikipedia) — one or more point releases ahead of this checkout.
- The runtime version string is the macro `dODE_VERSION`, generated into `include/ode/version.h` from `version.h.in:4` (`#define dODE_VERSION "@ODE_VERSION@"`) using the `ODE_VERSION` value from the build system.
- `CHANGELOG.txt` is append-only and sorted **newest-first** (newest entry 06/28/2023, oldest 09/20/2001); it tags changes by **date, not release number**. The last explicit "version bump" line is 0.10.1 (`CHANGELOG.txt:698`); later point releases are VCS-tagged only.
- ODE removes and renames API across its history. Deprecated-but-still-present symbols are flagged with the `ODE_API_DEPRECATED` marker macro (`odeconfig.h:58-62`); *removed* symbols are gone from headers entirely. Recommend the modern name in both cases.
- `grep -rn 'ODE_API_DEPRECATED' include/ode/*.h` enumerates the live deprecation surface for whatever tree you are in — trust the headers over any prose.

## When to use

Read this before recommending or generating any ODE call when you are unsure whether the symbol still exists, when a user pastes legacy ODE code (CCylinder, StepFast, GeomGroup, `dInitODE`), or when targeting a specific ODE version. It exists to stop an agent from emitting a removed or deprecated API.

## Public API (C)

### Version surface

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dODE_VERSION` | `#define dODE_VERSION "@ODE_VERSION@"` | include/ode/version.h.in:4 | Compile-time version string macro; expands to the build's `ODE_VERSION` (here `0.16`). Use for `#if`/string version checks. Header is *generated* — read the built `include/ode/version.h` to see the literal. |

### Deprecated-but-present (still compile; carry `ODE_API_DEPRECATED`)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomRaySetParams` | `ODE_API_DEPRECATED ODE_API void dGeomRaySetParams (dGeomID g, int FirstContact, int BackfaceCull);` | include/ode/collision.h:1078 | Deprecated combined ray-flag setter; replaced by the split `dGeomRaySetFirstContact`/`dGeomRaySetBackfaceCull`/`dGeomRaySetClosestHit`. |
| `dGeomRayGetParams` | `ODE_API_DEPRECATED ODE_API void dGeomRayGetParams (dGeomID g, int *FirstContact, int *BackfaceCull);` | include/ode/collision.h:1079 | Deprecated combined ray-flag getter; replaced by `dGeomRayGetFirstContact`/`dGeomRayGetBackfaceCull`/`dGeomRayGetClosestHit`. |
| `dCreateGeomTransform` and family | `ODE_API_DEPRECATED ODE_API dGeomID dCreateGeomTransform (dSpaceID space);` (+ `dGeomTransformSetGeom`/`GetGeom`/`SetCleanup`/`GetCleanup`/`SetInfo`/`GetInfo`) | include/ode/collision.h:1089-1095 | Deprecated geometry-transform wrapper; replaced by geom offsets (`dGeomSetOffsetPosition`/`dGeomSetOffsetRotation`). |
| `dGeomTriMeshDataPreprocess` | `ODE_API int dGeomTriMeshDataPreprocess(dTriMeshDataID g);` | include/ode/collision_trimesh.h:200 | Obsolete single-arg preprocess; equivalent to `dGeomTriMeshDataPreprocess2(g, (1U << dTRIDATAPREPROCESS_BUILD_CONCAVE_EDGES), NULL)`. |
| `dGeomTriMeshDataGetBuffer` | `ODE_API_DEPRECATED ODE_API void dGeomTriMeshDataGetBuffer(dTriMeshDataID g, unsigned char** buf, int* bufLen);` | include/ode/collision_trimesh.h:208 | Deprecated; use `dGeomTriMeshDataGet2` with `dTRIMESHDATA_USE_FLAGS`. |
| `dGeomTriMeshDataSetBuffer` | `ODE_API_DEPRECATED ODE_API void dGeomTriMeshDataSetBuffer(dTriMeshDataID g, unsigned char* buf);` | include/ode/collision_trimesh.h:209 | Deprecated; use `dGeomTriMeshDataSet` with `dTRIMESHDATA_USE_FLAGS`. |
| `dJointSetHinge2Axis1` | `ODE_API_DEPRECATED ODE_API void dJointSetHinge2Axis1 (dJointID j, dReal x, dReal y, dReal z);` | include/ode/objects.h:2094 | Deprecated as unsafe; use `dJointSetHinge2Axes` (sets both axes atomically). |
| `dJointSetHinge2Axis2` | `ODE_API_DEPRECATED ODE_API void dJointSetHinge2Axis2 (dJointID j, dReal x, dReal y, dReal z);` | include/ode/objects.h:2104 | Deprecated as unsafe; use `dJointSetHinge2Axes`. |
| `dJointSetPUAnchorDelta` | `ODE_API_DEPRECATED ODE_API void dJointSetPUAnchorDelta (dJointID, dReal x, dReal y, dReal z, ...);` | include/ode/objects.h:2280 | Deprecated; use `dJointSetPUAnchorOffset`. |
| `dJointSetPistonAxisDelta` | `ODE_API_DEPRECATED ODE_API void dJointSetPistonAxisDelta (dJointID j, dReal x, dReal y, dReal z, dReal ax, dReal ay, dReal az);` | include/ode/objects.h:2426 | Deprecated piston axis-delta setter. |
| `dMassSetCappedCylinder` | `ODE_API_DEPRECATED ODE_API void dMassSetCappedCylinder(dMass *a, dReal b, int c, dReal d, dReal e);` | include/ode/mass.h:84 | Deprecated legacy "capped cylinder" (=capsule) naming; use `dMassSetCapsule`. |
| `dMassSetCappedCylinderTotal` | `ODE_API_DEPRECATED ODE_API void dMassSetCappedCylinderTotal(dMass *a, dReal b, int c, dReal d, dReal e);` | include/ode/mass.h:85 | Deprecated; use `dMassSetCapsuleTotal`. |
| `dInitODE` | `ODE_API void dInitODE(void);` | include/ode/odeinit.h:95 | Obsolete init entry point (not marked with the macro, but doc-flagged obsolete); equivalent to `dInitODE2(0)` + `dAllocateODEDataForThread(dAllocateMaskAll)`. Use `dInitODE2`. |

### Back-compat `#define` aliases (CCylinder → Capsule)

| Symbol (old) | Definition | Source | Purpose |
| --- | --- | --- | --- |
| `dCreateCCylinder` | `#define dCreateCCylinder dCreateCapsule` | include/ode/collision.h:1056 | Old name compiles only via this macro; use `dCreateCapsule`. |
| `dGeomCCylinderSetParams` | `#define dGeomCCylinderSetParams dGeomCapsuleSetParams` | include/ode/collision.h:1057 | Use `dGeomCapsuleSetParams`. |
| `dGeomCCylinderGetParams` | `#define dGeomCCylinderGetParams dGeomCapsuleGetParams` | include/ode/collision.h:1058 | Use `dGeomCapsuleGetParams`. |
| `dGeomCCylinderPointDepth` | `#define dGeomCCylinderPointDepth dGeomCapsulePointDepth` | include/ode/collision.h:1059 | Use `dGeomCapsulePointDepth`. |
| `dCCylinderClass` | `#define dCCylinderClass dCapsuleClass` | include/ode/collision.h:1060 | Use `dCapsuleClass`. |

## Key rules

- This tree's version is **0.16** — read `configure.ac` (or the generated `include/ode/version.h`) for the authoritative string, not the CMake `SOVERSION` which is the ABI/libtool version, not the product version (configure.ac:2-3).
- The runtime version macro is `dODE_VERSION`, added 02/27/14 (issue #24) and generated from `version.h.in`; its literal value depends on the build (include/ode/version.h.in:4).
- The latest *published* release is **0.16.6** (2025-01-16); this checkout's `CHANGELOG.txt` ends at 06/28/2023, so 0.16.1–0.16.6 point-fixes are not recorded here — treat 0.16.6 as the canonical "current" release when advising. License is dual BSD / LGPL (https://en.wikipedia.org/wiki/Open_Dynamics_Engine).
- `dWorldStepFast1` was **removed** 09/05/09 along with `dWorldGetAutoEnableDepthSF1`/`dWorldSetAutoEnableDepthSF1`; use `dWorldStep` (full LCP) or `dWorldQuickStep` (iterative) — there is no StepFast symbol in current headers (CHANGELOG.txt:467).
- CCylinder was **renamed** to Capsule 03/17/06; the old `dCreateCCylinder` etc. survive only as `#define` aliases — emit the Capsule names (CHANGELOG.txt:940).
- `dInitODE` is obsolete; `dInitODE2` is the initialization entry point, and a bare `dInitODE` equals `dInitODE2(0)` then `dAllocateODEDataForThread(dAllocateMaskAll)` (include/ode/odeinit.h:83-91).
- `dWorldStep` and `dWorldQuickStep` were changed 06/14/09 to **return a boolean success status** — check the return; a step can fail (e.g. OOM) (CHANGELOG.txt:497-498).
- `dCloseODE()` (library shutdown) was added 06/23/02; always pair init with it at program end (CHANGELOG.txt:1188).
- The `[u]int[8/16/32]` global typedefs were **renamed** with a `d` prefix (`d[u]int...`) 02/03/13 to stop polluting the global namespace; unprefixed names are no longer provided (CHANGELOG.txt:264-269).
- `dInfinity` changed 04/18/04 from an exported library variable to a `#define` (`FLT_MAX`/`DBL_MAX`/`HUGE_VAL`) — it is a compile-time constant, there is no symbol to link (CHANGELOG.txt:1028-1032).
- The need to `#define dSINGLE`/`dDOUBLE` was removed 06/08/12 — precision is baked into the generated `ode/precision.h`; do not hand-define it inconsistently with the built library (CHANGELOG.txt:304-305).
- The old (pre-2002) collision system and `dGeomGroup*` were **removed** 04/18/04; `dGeomGroups` are equivalent to spaces — use `dSpace*` (CHANGELOG.txt:1034-1039).
- `dWorld[Get/Set]AutoDisableLinearAverageThreshold` and the `...AngularAverageThreshold` variants were **removed from public headers** 08/10/2014 (orphaned since rev.1052) — use the non-Average AutoDisable `*Threshold` accessors (CHANGELOG.txt:214-216).
- The `ODE_API_DEPRECATED` marker was introduced 07/15/08; it maps to `__declspec(deprecated)` (MSVC) or `__attribute__((__deprecated__))` (GCC/Clang) and nothing otherwise (include/ode/odeconfig.h:58-62).
- Version→feature mapping (from the ode.org wiki cross-check, since `CHANGELOG.txt` tags by date not release): CCylinder→Capsule = 0.6; `dInitODE2` multi-thread init = 0.10/0.12; `dWorldStepFast1` removal = 0.12; `dGeomRaySetParams` deprecation = 0.13.1; latest line = 0.16.x (https://ode.org/wiki/index.php?title=Changelog).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `dWorldStepFast1(world, stepsize, maxiterations);` | `dWorldQuickStep(world, stepsize); /* iterations via dWorldSetQuickStepNumIterations */` | `dWorldStepFast1` was removed 09/05/09 (CHANGELOG.txt:467); QuickStep is its iterative successor. |
| `dCreateCCylinder(space,r,l); dGeomCCylinderSetParams(g,r,l);` | `dCreateCapsule(space,r,l); dGeomCapsuleSetParams(g,r,l);` | CCylinder was renamed Capsule 03/17/06 (CHANGELOG.txt:940); CCylinder names are only `#define` aliases (collision.h:1056-1060). |
| `dInitODE(); /* ... */ dWorldStep(world,dt); // no return check` | `dInitODE2(0); /* ... */ if(!dWorldStep(world,dt)){ /* failed */ }` | `dInitODE` is obsolete (odeinit.h:83-91) and `dWorldStep`/`dWorldQuickStep` return a checkable bool since 06/14/09 (CHANGELOG.txt:497-498). |
| `dJointSetHinge2Axis1(j,x,y,z); dJointSetHinge2Axis2(j,x,y,z);` | `dJointSetHinge2Axes(j, axis1, axis2);` | The per-axis setters were deprecated as unsafe 06/03/16 (CHANGELOG.txt:148-150; objects.h:2094/2104); set both axes atomically. |
| `dGeomTriMeshDataGetBuffer(d,&buf,&len); dGeomTriMeshDataPreprocess(d);` | `dGeomTriMeshDataGet2(d, dTRIMESHDATA_USE_FLAGS, &sz); dGeomTriMeshDataPreprocess2(d, flags, NULL);` | `*Buffer` accessors are deprecated (collision_trimesh.h:208-209) and `Preprocess` is obsolete (collision_trimesh.h:198-200). |
| `dMassSetCappedCylinder(&m, density, dir, r, l);` | `dMassSetCapsule(&m, density, dir, r, l);` | `dMassSetCappedCylinder`/`Total` are deprecated capsule-naming (mass.h:84-85). |
| `#define dSINGLE` then `#include <ode/ode.h>` | `#include <ode/ode.h>` and match the lib's `ode/precision.h` | Precision moved to generated `ode/precision.h` 06/08/12 (CHANGELOG.txt:304-305); mismatched manual defines cause ABI/struct-size errors. |
| `extern dReal dInfinity; /* link an exported variable */` | `dReal big = dInfinity; /* it is a #define */` | `dInfinity` stopped being an exported variable 04/18/04 (CHANGELOG.txt:1028-1032); no symbol to link. |
| `uint32 flags; /* ODE's old unprefixed typedef */` | `duint32 flags; /* d-prefixed */` | `[u]int[8/16/32]` were renamed to the `d`-prefixed forms 02/03/13 (CHANGELOG.txt:264-269). |

## Deprecations / renames (old → new)

| Old (do not use) | New | Since | Source |
| --- | --- | --- | --- |
| `dWorldStepFast1` | `dWorldQuickStep` / `dWorldStep` | removed 09/05/09 | CHANGELOG.txt:467 |
| `dWorldGetAutoEnableDepthSF1` / `dWorldSetAutoEnableDepthSF1` | `dWorldSetQuickStepNumIterations` + AutoDisable params | removed 09/05/09 | CHANGELOG.txt:467 |
| `dWorldGetAutoDisableLinearAverageThreshold` / `...SetAutoDisableLinearAverageThreshold` | `dWorldGet/SetAutoDisableLinearThreshold` (non-Average) | removed 08/10/2014 | CHANGELOG.txt:214-216 |
| `dWorldGetAutoDisableAngularAverageThreshold` / `...SetAutoDisableAngularAverageThreshold` | `dWorldGet/SetAutoDisableAngularThreshold` (non-Average) | removed 08/10/2014 | CHANGELOG.txt:214-216 |
| `dCreateCCylinder` | `dCreateCapsule` | renamed 03/17/06 | include/ode/collision.h:1056 |
| `dGeomCCylinderSetParams` | `dGeomCapsuleSetParams` | renamed 03/17/06 | include/ode/collision.h:1057 |
| `dGeomCCylinderGetParams` | `dGeomCapsuleGetParams` | renamed 03/17/06 | include/ode/collision.h:1058 |
| `dGeomCCylinderPointDepth` | `dGeomCapsulePointDepth` | renamed 03/17/06 | include/ode/collision.h:1059 |
| `dCCylinderClass` | `dCapsuleClass` | renamed 03/17/06 | include/ode/collision.h:1060 |
| `dMassSetCappedCylinder` | `dMassSetCapsule` | deprecated | include/ode/mass.h:84 |
| `dMassSetCappedCylinderTotal` | `dMassSetCapsuleTotal` | deprecated | include/ode/mass.h:85 |
| `dInitODE` | `dInitODE2` | obsolete | include/ode/odeinit.h:83-95 |
| `dGeomRaySetParams` | `dGeomRaySetFirstContact` + `dGeomRaySetBackfaceCull` (+ `dGeomRaySetClosestHit`) | deprecated (0.13.1) | include/ode/collision.h:1078 |
| `dGeomRayGetParams` | `dGeomRayGetFirstContact` + `dGeomRayGetBackfaceCull` (+ `dGeomRayGetClosestHit`) | deprecated (0.13.1) | include/ode/collision.h:1079 |
| `dJointSetHinge2Axis1` | `dJointSetHinge2Axes` | deprecated 06/03/16 | include/ode/objects.h:2094 |
| `dJointSetHinge2Axis2` | `dJointSetHinge2Axes` | deprecated 06/03/16 | include/ode/objects.h:2104 |
| `dJointSetPUAnchorDelta` | `dJointSetPUAnchorOffset` | deprecated 11/20/08 | include/ode/objects.h:2280 |
| `dJointSetPistonAxisDelta` | (deprecated; no offset rename) | deprecated | include/ode/objects.h:2426 |
| `dJointSetHingeAxisDelta` | `dJointSetHingeAxisOffset` | renamed 07/08/08 | CHANGELOG.txt:721-723 |
| `dGeomTriMeshDataGetBuffer` | `dGeomTriMeshDataGet2` with `dTRIMESHDATA_USE_FLAGS` | deprecated | include/ode/collision_trimesh.h:208 |
| `dGeomTriMeshDataSetBuffer` | `dGeomTriMeshDataSet` with `dTRIMESHDATA_USE_FLAGS` | deprecated | include/ode/collision_trimesh.h:209 |
| `dGeomTriMeshDataPreprocess` | `dGeomTriMeshDataPreprocess2` | obsolete | include/ode/collision_trimesh.h:200 |
| `dCreateGeomTransform` / `dGeomTransform*` family | geom offsets `dGeomSetOffsetPosition` / `dGeomSetOffsetRotation` | deprecated | include/ode/collision.h:1089-1095 |
| `dGeomGroup*` (`dCreateGeomGroup` etc.) | spaces (`dSpace*`) | removed 04/18/04 | CHANGELOG.txt:1038-1039 |
| `dJointAddPUTorque` | `dJointAddPUTorques` | replaced 06/28/2023 | CHANGELOG.txt:12-13 |
| `uint8`/`uint16`/`uint32`/`int8`/`int16`/`int32` (ODE's unprefixed typedefs) | `duint8`/`duint16`/`duint32`/`dint8`/`dint16`/`dint32` | renamed 02/03/13 | CHANGELOG.txt:264-269 |
| `dInfinity` (exported variable) | `dInfinity` (compile-time `#define`) | changed 04/18/04 | CHANGELOG.txt:1028-1032 |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dODE_VERSION` | `"@ODE_VERSION@"` → `"0.16"` in this tree | include/ode/version.h.in:4 | Version string macro; resolved at build from `configure.ac:3` (`ODE_VERSION=0.16`). [VERIFY] the literal in a built tree by reading the generated `include/ode/version.h` (not committed in this checkout). |
| `ODE_API_DEPRECATED` | `__declspec(deprecated)` (MSVC) / `__attribute__((__deprecated__))` (GCC/Clang) / empty | include/ode/odeconfig.h:58-62 | Marker macro flagging a function deprecated (added 07/15/08, CHANGELOG.txt:690-691). `grep` it to enumerate the live deprecation surface. |

## API milestone timeline (notable, newest-first)

- **06/28/2023** — `dJointAddPUTorques()` added to replace the orphaned `dJointAddPUTorque` declaration (CHANGELOG.txt:12-13). *(newest entry in this tree's CHANGELOG)*
- **02/21/2019** — Dynamic QuickStep iteration-count adjustment (`dWorldSet/GetQuickStepDynamicIterationParameters`) (CHANGELOG.txt:50).
- **01/20/2019** — x86 targets emit SSE2 by default (CHANGELOG.txt:88-89 region).
- **10/09/2017** — CMake build support added (CHANGELOG.txt:88).
- **02/19/2017 / 02/20/2017** — `dWorldStep` threading extended to force-application/position-update; built-in multithreading default-on. **WARNING:** with threading enabled, body callbacks may run on multiple threads.
- **11/13/2016** — `dGeomTriMeshDataGetBuffer`/`SetBuffer` deprecated; use Set/Get2 with `dTRIMESHDATA_USE_FLAGS` (collision_trimesh.h:208-209).
- **06/03/2016** — `dJointSetHinge2Axes` added; `dJointSetHinge2Axis1/2` deprecated as unsafe (CHANGELOG.txt:147-150).
- **08/10/2014** — `dWorld...AutoDisable[Linear/Angular]AverageThreshold` declarations removed from headers (CHANGELOG.txt:214-216).
- **02/27/2014** — `dODE_VERSION` macro added (issue #24) (CHANGELOG.txt:223).
- **02/03/2013** — `[u]int[8/16/32]` typedefs renamed with `d` prefix (CHANGELOG.txt:264-269).
- **03/17/2012** — Threaded-execution interface + optional built-in threading implementation added (CHANGELOG.txt:327).
- **06/08/2012** — Need to `#define dSINGLE`/`dDOUBLE` removed; precision moved to generated `ode/precision.h` (CHANGELOG.txt:304-305).
- **09/05/2009** — `dWorldStepFast1` (and `AutoEnableDepthSF1` accessors) removed (CHANGELOG.txt:467).
- **06/14/2009** — `dWorldStep`/`dWorldQuickStep` changed to return a boolean success status; QuickStep re-implemented to drop stack allocation (CHANGELOG.txt:497-498).
- **07/15/2008** — `ODE_API_DEPRECATED` marker macro introduced (CHANGELOG.txt:690-691).
- **07/08/2008** — `dJointSetHingeAxisDelta` renamed to `dJointSetHingeAxisOffset` (CHANGELOG.txt:721-723).
- **10/26/2006** — `dInitODE` introduced; trimesh backend refactored OPCODE→GIMPACT (CHANGELOG.txt:889, :901).
- **03/17/2006** — CCylinder renamed Capsule (CHANGELOG.txt:940).
- **04/18/2004** — Old collision system + `dGeomGroups` removed; `dInfinity` became a `#define` (CHANGELOG.txt:1028, :1034-1039).
- **05/18/2004** — QuickStep solver (`dWorldQuickStep`) added.
- **06/23/2002** — `dCloseODE()` added (CHANGELOG.txt:1188).
- **09/20/2001** — oldest CHANGELOG entry.

Released-line cross-check (web, not in this tree's CHANGELOG): newest published release is **0.16.6** (2025-01-16); point releases 0.16.1–0.16.6 are VCS-tagged upstream (bitbucket.org/odedevs/ode; mirrored on sourceforge.net/projects/opende) and not recorded as version-bump lines here.

## Things to never invent

These symbols are removed, renamed, or never existed in the cited form — never emit them; recommend the replacement.

- `dWorldStepFast1` → REMOVED 09/05/09; use `dWorldQuickStep` or `dWorldStep` (CHANGELOG.txt:467).
- `dWorldStepFast` → never existed as a public symbol; the removed name was `dWorldStepFast1` (CHANGELOG.txt:467).
- `dWorldGetAutoEnableDepthSF1` / `dWorldSetAutoEnableDepthSF1` → REMOVED 09/05/09 with StepFast1 (CHANGELOG.txt:467).
- `dWorldGetAutoDisableLinearAverageThreshold` / `dWorldSetAutoDisableLinearAverageThreshold` → REMOVED from headers 08/10/2014 (CHANGELOG.txt:214-216).
- `dWorldGetAutoDisableAngularAverageThreshold` / `dWorldSetAutoDisableAngularAverageThreshold` → REMOVED from headers 08/10/2014 (CHANGELOG.txt:214-216).
- `dCreateCCylinder` → `dCreateCapsule` (old name only a macro alias, collision.h:1056).
- `dGeomCCylinderSetParams` → `dGeomCapsuleSetParams` (collision.h:1057).
- `dGeomCCylinderGetParams` → `dGeomCapsuleGetParams` (collision.h:1058).
- `dGeomCCylinderPointDepth` → `dGeomCapsulePointDepth` (collision.h:1059).
- `dCCylinderClass` → `dCapsuleClass` (collision.h:1060).
- `dMassSetCappedCylinder` → `dMassSetCapsule` (deprecated, mass.h:84).
- `dMassSetCappedCylinderTotal` → `dMassSetCapsuleTotal` (deprecated, mass.h:85).
- `dInitODE` → `dInitODE2` (`dInitODE` obsolete, odeinit.h:83-95).
- `dGeomRaySetParams` → `dGeomRaySetFirstContact` + `dGeomRaySetBackfaceCull` (+ `dGeomRaySetClosestHit`); deprecated, collision.h:1078.
- `dGeomRayGetParams` → `dGeomRayGetFirstContact` + `dGeomRayGetBackfaceCull`; deprecated, collision.h:1079.
- `dJointSetHinge2Axis1` → `dJointSetHinge2Axes` (deprecated as unsafe, objects.h:2094; CHANGELOG.txt:148-150).
- `dJointSetHinge2Axis2` → `dJointSetHinge2Axes` (deprecated as unsafe, objects.h:2104).
- `dJointSetPUAnchorDelta` → `dJointSetPUAnchorOffset` (deprecated, objects.h:2280; CHANGELOG.txt:613-614).
- `dJointSetPistonAxisDelta` → deprecated (objects.h:2426).
- `dJointSetHingeAxisDelta` → `dJointSetHingeAxisOffset` (renamed 07/08/08; the Delta name never shipped stable) (CHANGELOG.txt:721-723).
- `dGeomTriMeshDataGetBuffer` → `dGeomTriMeshDataGet2` with `dTRIMESHDATA_USE_FLAGS` (deprecated, collision_trimesh.h:208).
- `dGeomTriMeshDataSetBuffer` → `dGeomTriMeshDataSet` with `dTRIMESHDATA_USE_FLAGS` (deprecated, collision_trimesh.h:209).
- `dGeomTriMeshDataGetBuff` / `dGeomTriMeshDataSetBuff` → `dGeomTriMeshDataGet` / `dGeomTriMeshDataSet` (deprecated 11/13/2016, CHANGELOG.txt:125-128).
- `dGeomTriMeshDataPreprocess` → `dGeomTriMeshDataPreprocess2` (obsolete single-arg form, collision_trimesh.h:198-200).
- `dCreateGeomTransform` / `dGeomTransformSetGeom` / `dGeomTransformGetGeom` / `dGeomTransformSetCleanup` / `dGeomTransformGetCleanup` / `dGeomTransformSetInfo` / `dGeomTransformGetInfo` → use geom offsets `dGeomSetOffset*` (deprecated, collision.h:1089-1095).
- `dGeomGroup*` (`dCreateGeomGroup` etc.) → REMOVED 04/18/04; use spaces (`dSpace*`) (CHANGELOG.txt:1038-1039).
- `ODE_OLD_COLLISION` config flag → meaningless since old collision system removed 04/18/04 (CHANGELOG.txt:1034-1036).
- `dJointAddPUTorque` → `dJointAddPUTorques` (orphaned declaration replaced 06/28/2023, CHANGELOG.txt:12-13).
- `uint8`/`uint16`/`uint32`/`int8`/`int16`/`int32` (ODE's old unprefixed typedefs) → `duint8`/`duint16`/`duint32`/`dint8`/`dint16`/`dint32` (renamed 02/03/13, CHANGELOG.txt:264-269).
- `dWorldStepFast1` maxiterations / AutoEnableDepth concepts → replaced by `dWorldSetQuickStepNumIterations` + AutoDisable params.

### [VERIFY] markers (ungrounded facts flagged inline above)

- `dODE_VERSION` literal in a built tree — `include/ode/version.h` is generated and not committed in this checkout; only `version.h.in:4` (the `@ODE_VERSION@` template) is confirmed. The `0.16` value is grounded in `configure.ac:3`.
- Replacement signatures `dMassSetCapsule`, `dMassSetCapsuleTotal`, `dGeomSetOffsetPosition`, `dGeomSetOffsetRotation`, `dJointSetHinge2Axes`, `dJointSetPUAnchorOffset`, `dJointSetHingeAxisOffset`, `dWorldSetQuickStepNumIterations` are named from the inventory's old→new mappings; the deprecated *old* symbols are header-confirmed here, but exact modern signatures should be grep-confirmed in `include/ode/{mass.h,collision.h,objects.h}` before emitting.
- `dGeomTriMeshDataGetBuff`/`SetBuff` and the 0.13.1 / 0.16.6 / version-tag mappings come from the inventory's web/wiki cross-check, not from this tree's headers — confirm against the upstream repo or wiki before asserting a specific release tag.

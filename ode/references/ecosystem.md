# ODE ecosystem map — "what else is in the repo"

This is the orientation map for everything in the ODE source tree that is **not**
the public `include/ode/*.h` C API (and not the `include/drawstuff/*.h` harness).
Use it to know what a directory is, why it exists, and — for the parts you must
never call from user code — where the boundary is.

> **Boundary rule (load-bearing).** The collision/OS backends (`OPCODE/`,
> `GIMPACT/`, `libccd/`, `ou/`) live **outside `include/ode`** and are compiled
> into `libode` as private internals. User code must interact with them **only**
> through the public ODE C API (`dGeomTriMesh*`, `dCollide`, etc.). Their headers
> are not installed and their symbols carry no API/ABI stability guarantee.
> Cite: `CMakeLists.txt:318-383`.

---

## Collision/OS backends

These four are **bundled third-party libraries** selected at build time. They are
ODE internals — listed here so you recognize them and choose them correctly via
build flags, not so you call them.

### OPCODE — default trimesh-trimesh backend
- **What:** ODE's DEFAULT triangle-mesh collision backend (OPtimized COllision
  DEtection, by Pierre Terdiman). Uses hierarchical (no-leaf, quantizable)
  AABB-trees; memory-efficient and fast for mostly-static meshes; supports
  mesh-mesh plus sphere/box/ray/OBB/planes queries.
  Cite: `OPCODE/README-ODE.txt`, `OPCODE/ReadMe.txt`.
- **Where / role:** `OPCODE/` (entry header `OPCODE/Opcode.h`; `OPCODE/Ice/` has
  the support files; `OPCODE/README-ODE.txt`, `OPCODE/ReadMe.txt`). It is an
  internal backend reached only via ODE's `dGeomTriMesh*` API.
- **Build flag:** selected by `ODE_WITH_OPCODE=ON` (the default) / autotools
  `--with-trimesh=opcode`; sets define `-DdTRIMESH_OPCODE`.
  Cite: `CMakeLists.txt:660` (`dTRIMESH_OPCODE` — "Compile define set when ODE_WITH_OPCODE is ON (the default). Selects OPCODE as the trimesh-trimesh collider backend. Paired with dTRIMESH_ENABLED.").
- **Implication:** stable default for trimesh collision with **no scene-size
  limit**, but optimized for static geometry (recomputes transformed vertices
  each query for dynamic meshes). The legacy trimesh-trimesh collider is opt-in
  via `-DdTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER` (`--enable-old-trimesh`),
  OPCODE only.
  Cite: `CMakeLists.txt:662-664` ("ODE_OLD_TRIMESH ... only applies to the OPCODE backend; it selects the legacy trimesh-trimesh collider. Autotools: --enable-old-trimesh (ignored unless trimesh=opcode).").

### GIMPACT — alternative (experimental) trimesh backend
- **What:** ALTERNATIVE trimesh backend (Geometric tools for VR, by Francisco
  Leon), marked experimental in ODE. No hierarchy trees; AABB-tree for rigid
  meshes OR box-pruning for deformable meshes, with cheap updates on
  transform/vertex edits — better for moving/deformable trimeshes.
  Cite: `GIMPACT/include/GIMPACT/gimpact.h`.
- **Where / role:** `GIMPACT/` (entry header `GIMPACT/include/GIMPACT/gimpact.h`
  with `gimpact_init`/`gimpact_terminate`; sources in `GIMPACT/src/`). Internal
  backend; use via ODE's trimesh API only.
- **Build flag:** `ODE_WITH_GIMPACT=ON` (default OFF) / `--with-trimesh=gimpact`;
  sets define `-DdTRIMESH_GIMPACT`. Mutually exclusive with OPCODE.
  Cite: `CMakeLists.txt:598` (`dTRIMESH_GIMPACT` — "Compile define set when ODE_WITH_GIMPACT is ON. Selects GIMPACT instead of OPCODE for trimesh collisions. Mutually exclusive with dTRIMESH_OPCODE in a single build.").
- **Implication (hard limit):** scene coordinates are HARD-LIMITED to
  `[-1638, +1638]` (`MAX_AABB_SIZE`, the box-pruning spatial-sort cap); geometry
  outside this range can mis-collide. **Do not build very large worlds (>~3276
  units span) with GIMPACT — choose OPCODE for large worlds.**
  Cite: `GIMPACT/include/GIMPACT/gim_boxpruning.h:262` (`MAX_AABB_SIZE (1638.0f)` — "INTERNAL GIMPACT limit. Box-pruning spatial sort caps usable scene coordinates to [-1638, +1638]; ... do NOT build very large worlds (>~3276 units span) with the GIMPACT backend.").

### libccd — opt-in convex-collision backend
- **What:** Convex-shape collision library (by Daniel Fiser) implementing GJK
  (intersect/distance), EPA (penetration depth) and MPR/XenoCollide. ODE uses it
  to FILL IN collider pairs it lacks robust native code for. Each geom type plugs
  in via a "support function" (returns the furthest point of a convex object in a
  direction).
  Cite: `libccd/src/ccd/ccd.h`.
- **Where / role:** `libccd/` (entry header `libccd/src/ccd/ccd.h`; sources
  `alloc.c`/`ccd.c`/`mpr.c`/`polytope.c`/`support.c`/`vec3.c`). Internal backend,
  not a public ODE symbol (e.g. `ccdGJKIntersect`, `ccd_t` are not ODE API).
  Cite: `libccd/src/ccd/ccd.h` (`ccdGJKIntersect` — "INTERNAL backend (libccd, outside include/ode). ... ODE calls this (and EPA/MPR siblings) internally when built with -DdLIBCCD_ENABLED ... Not a public ODE symbol.").
- **Build flag:** `ODE_WITH_LIBCCD=ON` (default OFF), bundled (INTERNAL) or system
  (`find_package(ccd)`); sets `-DdLIBCCD_ENABLED` plus per-pair sub-defines.
  Autotools `--enable-libccd` enables all libccd colliders **except box-cylinder**.
  Cite: `CMakeLists.txt:609` (`dLIBCCD_ENABLED / dLIBCCD_INTERNAL / dLIBCCD_SYSTEM` — per-pair sub-defines `dLIBCCD_BOX_CYL, dLIBCCD_CAP_CYL, dLIBCCD_CYL_CYL, dLIBCCD_CONVEX_BOX, dLIBCCD_CONVEX_CAP, dLIBCCD_CONVEX_CONVEX, dLIBCCD_CONVEX_CYL, dLIBCCD_CONVEX_SPHERE`).
- **Implication:** enable libccd to get reliable cylinder/convex collisions (box-cylinder,
  capsule-cylinder, cylinder-cylinder, convex-box, convex-capsule, convex-convex,
  convex-cylinder, convex-sphere). Its precision MUST track ODE's: CMake sets
  `CCD_PRECISION = CCD_DOUBLE/CCD_SINGLE` in tandem with `ODE_DOUBLE_PRECISION`;
  a mismatch is a configuration error.
  Cite: `CMakeLists.txt:142` ("libccd precision must match ODE precision: CMake sets CCD_PRECISION = CCD_DOUBLE/CCD_SINGLE in tandem with ODE_DOUBLE_PRECISION via libccd/src/ccd/precision.h.in. Mismatched precision is a configuration error.").

### ou — portability/concurrency support library (NOT collision)
- **What:** ou (ODER's Utilities Library, by Oleh Derevenko) — **NOT a collision
  backend**. It is ODE's portability/concurrency support library: cross-platform
  atomic (interlocked) operations, thread-local storage, assertion macros,
  flags/enum templates.
  Cite: `ou/README.TXT`.
- **Where / role:** `ou/` (public headers `ou/include/ou/*.h`: `atomic.h`,
  `threadlocalstorage.h`, `atomicflags.h`, etc.; `ou/README.TXT`). Compiled under
  the C++ namespace `odeou` so multiple OU copies can link into one binary.
  Cite: `CMakeLists.txt:117` (`_OU_NAMESPACE = odeou` — "ou library is compiled into ODE under the C++ namespace 'odeou' so it can coexist with other copies of OU in the same binary. _OU_FEATURE_SET selects BASICS / ATOMICS / TLS tiers.").
- **Build flag:** gated by `ODE_WITH_OU` and the threading interface; sets
  `-DdATOMICS_ENABLED` / `-DdTLS_ENABLED`.
  Cite: `CMakeLists.txt:674-678` (`dATOMICS_ENABLED / dTLS_ENABLED` — "dATOMICS_ENABLED set when threading interface is on; dTLS_ENABLED additionally set when ODE_WITH_OU is ON, enabling thread-local-storage-backed global caches so separate spaces can be collision-checked from different threads.").
- **Implication:** building with `ODE_WITH_OU` (TLS) lets you run collision checks
  on SEPARATE `dSpace`s from different threads safely; without it those global
  caches are not thread-isolated.
  Cite: `ou/README.TXT:5` ("ou (ODER's Utilities) provides atomics + thread-local storage; ODE_WITH_OU enables TLS-backed global caches so separated spaces can be collision-checked concurrently. Compiled under namespace 'odeou' so multiple OU copies can link into one binary.").

### Where backends meet the engine (ODE-side glue)
Thin ODE-internal adapters live in `ode/src/collision_trimesh_opcode.*`,
`ode/src/collision_trimesh_gimpact.*`, `ode/src/collision_trimesh_internal.h`,
`ode/src/collision_trimesh_disabled.cpp`, `ode/src/odeou.*`. The public surface
they sit behind is `include/ode/collision_trimesh.h`. `collision_trimesh_disabled.cpp`
is the no-op path when `ODE_NO_TRIMESH` (then `dGeomTriMesh*` functions compile
out / silently do nothing — trimesh geoms collide with nothing).
Cite: `OPCODE/README-ODE.txt:6` ("If NEITHER trimesh backend is compiled (ODE_NO_TRIMESH / --with-trimesh=none), dGeomTriMesh* functions are compiled out / silently do nothing - trimesh geoms collide with nothing.").

> Trimesh backend selection is **mutually exclusive**: one of `-DdTRIMESH_OPCODE`
> (default) or `-DdTRIMESH_GIMPACT` per build; never both.
> Cite: `CMakeLists.txt:37-41` ("Trimesh backend is a build-time choice, mutually exclusive: ODE_WITH_OPCODE (default ON, -DdTRIMESH_OPCODE) OR ODE_WITH_GIMPACT (default OFF/experimental, -DdTRIMESH_GIMPACT). With autotools use --with-trimesh=opcode|gimpact|none (default opcode).").

**These backends are outside the public `include/ode` surface — do not `#include`
or call `Opcode::*`, `ccd*`, `gimpact_*`, or `odeou::*` from application code.**

---

## Tools (`tools/`)

`ls tools/` →
```
README.txt          # release-process instructions (build an ODE release)
msw-release.bat     # ODE Windows binary release script
src-release.sh      # ODE source-code release script
ode_convex_export.py# Blender exporter -> ODE Convex Geom C header
```

- **`README.txt`** documents the release process (prerequisites: Windows + POSIX
  system, Visual Studio, Subversion, 7zip, Doxygen on PATH). Cite: `tools/README.txt:1`
  ("HOW TO MAKE A RELEASE").
- **`msw-release.bat`** — Windows binary release script. Cite: `tools/msw-release.bat:2`
  ("ODE Windows Binary Release Script").
- **`src-release.sh`** — source-code release script. Cite: `tools/src-release.sh:3`
  ("ODE Source Code Release Script").
- **`ode_convex_export.py`** — a Blender (`#!BPY`, Blender 244) export script:
  "Export to Open Dynamics." It writes a Blender scene as ODE Convex Geom data
  (pointcount / planecount / `dReal *_points[]`) into a C header. Cite:
  `tools/ode_convex_export.py:13` ("This script Exports a Blender scene as a series of ODE Convex Geom data stored in a C header file.").

### DIF (Dynamics Interchange Format) — the in-API export tool
DIF export is part of the **public** API, not the `tools/` dir: the header
`include/ode/export-dif.h` declares one function:

```c
ODE_API void dWorldExportDIF (dWorldID w, FILE *file, const char *world_name);
```
Cite: `include/ode/export-dif.h:33` (`ODE_API void dWorldExportDIF (dWorldID w, FILE *file, const char *world_name);`). Call it to dump an entire world (bodies/joints) to a `FILE*` in DIF for inspection or regression diffing.

---

## Language bindings (`bindings/`)

`ls bindings/` → only one binding ships in-tree:
```
python/
```

### Python (`bindings/python/`)
`ls bindings/python/` →
```
demos/        INSTALL.txt   ode.pxd   ode.pyx   setup.py   TODO.txt
```
- **Kind:** a Cython wrapper around the ODE C library (derived from PyODE).
  `ode.pyx`/`ode.pxd` are Cython sources; the module is named `ode`. Cite:
  `bindings/python/ode.pyx:2` ("Python Open Dynamics Engine Wrapper").
- **Build:** `setup.py` is a distutils/Cython build that requires Cython and
  locates ODE via `pkg-config --cflags/--libs ode`. Cite: `bindings/python/setup.py:8`
  ("from Cython.Distutils import build_ext").
- **Requirements:** Python 2.4+ (tested 2.7), Cython 0.14.1+, an ODE shared
  library (or static with `-fPIC`), and `pkg-config`. Cite:
  `bindings/python/INSTALL.txt:3` ("1. Python 2.4 or higher (http://www.python.org/)").
- **Demos:** `demos/tutorial1.py`, `tutorial2.py`, `tutorial3.py`. Cite: `ls bindings/python/demos/`.

> A second, separate .NET binding exists under `contrib/` (`Ode.NET`,
> `DotNetManaged`) — see the contrib section; those are contributed, not the
> maintained in-tree binding.

---

## contrib (notable contributed tools)

`ls contrib/` →
```
README                  GeomTransformGroup      OdeModelProcessor
BreakableJoints         InteractiveCollisions   TerrainAndCone
dCylinder               Mac_CFMCarbon
DotNetManaged           Ode.NET
dRay
```
Cite: `ls contrib/`. Per `contrib/README`, this directory holds user-contributed
ODE-related code that is not integrated into the core tree; **no guarantees** —
it may be undocumented, untested, or may not compile. Cite: `contrib/README:1`
("This directory contains ODE-related things that have been generously contributed by ODE's users.").

Notable packages:
- **BreakableJoints** — adds breakable joints (a joint breaks when force is too
  high); ships a modified `test_buggy.cpp` (`test_breakable.cpp`) demo. Cite:
  `contrib/BreakableJoints/README:1` ("Breakable Joints").
- **dCylinder** — a cylinder geom contribution. Flagged unreliable: "WARNING:
  THIS IS NOT VERY RELIABLE CODE. IT HAS BUGS." Cite: `contrib/dCylinder/readme.txt:3`.
- **DotNetManaged** — a managed C++ (.NET) wrapper (`Body.cpp`, `DotNetManaged.sln`,
  `.vcproj`). Cite: `ls contrib/DotNetManaged/`.
- **dRay** — an early `dRay` collision class (interacts with `dPlane`, `dSphere`,
  `dBox`, `dCCylinder`; generates only the `pos` member). Cite:
  `contrib/dRay/README:6` ("Yesterday and today i've written a dRay class.").
- **GeomTransformGroup** — a patch adding the `dGeomTransformGroup` object to the
  geometry list (Tim Schmidt). Cite: `contrib/GeomTransformGroup/README:1`
  ("README for GeomTransformGroup by Tim Schmidt.").
- **InteractiveCollisions** — a GUI tool to test pairs of ODE geoms; uses
  AntTweakBar for the GUI and GLFW for windowing/input. Cite:
  `contrib/InteractiveCollisions/README:1` ("This is a simple tool to test pairs of ODE geoms.").
- **Mac_CFMCarbon** — a Mac CFM/Carbon (CodeWarrior, MacOS8/9 + OSX) port (Frank
  Condello). Cite: `contrib/Mac_CFMCarbon/README:2` ("ODE - Mac CFM Carbon Port").
- **Ode.NET** — .NET bindings for ODE (Jason Perkins). Cite: `contrib/Ode.NET/README:1`
  ("Ode.NET - .NET bindings for ODE").
- **OdeModelProcessor** — "The ODE Model Processor" (University of Otago, Richard
  Barrington); ships `OdeModelProcessor.sln`. Cite: `contrib/OdeModelProcessor/README.TXT:1`
  ("The ODE Model Processor").
- **TerrainAndCone** — heightfield terrain (`dTerrainZ` z-up / `dTerrainY` y-up,
  placeable, finite or infinite) and cone geoms with collision + drawing (Benoit
  Chaperot); ships `test_boxstackb.cpp`. Cite: `contrib/TerrainAndCone/readme.txt:1`
  ("Benoit CHAPEROT 2003-2004 www.jstarlab.com / Support for terrain and cones, collision and drawing.").

> contrib code is **not** part of the supported public API and is not built by the
> default ODE build; treat it as reference/patches, not as `include/ode` surface.

---

## External resources & corroborated practices

Community-corroborated rules for *using* the public API (sourced from the ODE
wiki/FAQ/user guide and tutorials in `web-best-practices.json`). These are
behavioral guidance, not new symbols.

**Main-loop & stepping**
- Run each frame in fixed order: `dSpaceCollide(space, data, &nearCallback)` →
  `dWorldStep`/`dWorldQuickStep(world, dt)` → `dJointGroupEmpty(contactgroup)`.
  Contact joints are temporary — empty the group every step or they accumulate
  and corrupt the sim. Cite: `https://bedroomcoders.co.uk/posts/151`.
- Use a CONSTANT (fixed) timestep; for variable frame rates use the accumulator
  pattern (`cache += elapsed; while (cache >= timestep) { step; cache -= timestep; }`).
  Cite: `https://ode.org/wiki/index.php/HOWTO_fixed_vs_variable_timestep`.
- Choose the stepper by scene size: `dWorldStep` (Dantzig LCP) near-exact, better
  for small/near-singular scenes, O(n)+O(m^2) stack and ~10x slower;
  `dWorldQuickStep` (SOR PGS) faster and scales, less accurate — tune with
  `dWorldSetQuickStepNumIterations` (default 20). Cite: `https://ode.org/wiki/index.php/Manual`.

**Stability knobs**
- First fix for near-singular/misbehaving sims: raise global CFM
  (`dWorldSetCFM`) to a small positive value; never negative. Cite: `https://ode.org/wiki/index.php/Manual`.
- Keep ERP at the default 0.2 (range 0.1-0.8); ERP=1 can cause instability. Cite:
  `https://ode.org/wiki/index.php/Manual`.
- Stop contact jitter with a small contact surface layer
  (`dWorldSetContactSurfaceLayer`, e.g. 0.001; default 0). Cite: `https://ode.org/wiki/index.php/Manual`.
- Most resting objects need at least three contacts to be stable; pass an array
  (`dContact contact[N]`) to `dCollide`, not a single struct. Cite:
  `https://www.ode.org/wiki/index.php?title=FAQ`.
- Enable auto-disable (and damping) on the WORLD before creating bodies so new
  bodies inherit it; a disabled body jointed to an enabled one re-enables next
  step. Cite: `https://ode.org/wiki/index.php/Manual`.
- Avoid large mass ratios between jointed/contacting bodies — `J*inv(M)*J'` becomes
  ill-conditioned and QuickStep's PGS diverges (explosions). Cite:
  `https://classic.gazebosim.org/tutorials?tut=physics_params`.

**Geometry & mass**
- Use `dCreatePlane` for infinite ground (bodies cannot tunnel through it), not a
  thin box or trimesh slab. Cite: `https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=2594`.
- Set a realistic total mass (e.g. `dMassSetBoxTotal` / `dMassAdjust`), not
  density=1 (1 kg/m^3 is lighter than air). Cite:
  `https://www.gamedev.net/forums/topic/571803-ode-collision-joint-problems/`.
- Body build order: `dBodyCreate` → `dBodySetPosition`/`Rotation` → build `dMass`
  + `dBodySetMass` → `dCreate<shape>` → `dGeomSetBody`; set positions BEFORE
  joints. Cite: `https://compusciencing.github.io/starting-with-ode.html`.
- Do NOT fake a static object with huge mass + per-step pose reset — create the
  geom with body ID 0 instead. Cite: `https://ode.org/wiki/index.php/Manual`.

**Vehicles & contacts**
- Connect each wheel to the chassis with a hinge-2 joint (`dJointCreateHinge2`):
  axis1 = steer/suspension, axis2 = wheel spin; suspension via
  `dJointSetHinge2Param` with `dParamSuspensionERP`/`dParamSuspensionCFM`; lock
  non-steering wheels with `dParamLoStop = dParamHiStop = 0`. Cite:
  `http://ode.org/wikiold/htmlfile15.html`.
- Filter unwanted pairs in `nearCallback` before `dCollide` (the buggy demo uses
  an XOR ground test `if (!(g1 ^ g2)) return;`). Cite:
  `https://github.com/vancegroup-mirrors/open-dynamics-engine-svnmirror/blob/master/ode/demo/demo_buggy.cpp`.
- Derive soft contact/suspension from a spring (kp) + damper (kd) + timestep h:
  `soft_erp = h*kp/(h*kp+kd)`, `soft_cfm = 1/(h*kp+kd)`, applied via
  `dContactSoftERP|dContactSoftCFM` in `surface.mode`. Cite:
  `https://classic.gazebosim.org/tutorials?tut=physics_params`.

**Common crashes / mistakes (from `web-best-practices.json` mistakes)**
- Sizing the `dContact` buffer smaller than the max-contacts arg to `dCollide`
  writes past the buffer → crash; size `dContact contact[N]` and pass stride
  `sizeof(dContact)`. Cite: `https://bedroomcoders.co.uk/posts/151`.
- `dWorldStep` on a large scene can overflow the (small Windows) stack — increase
  stack size or switch to `dWorldQuickStep`. Cite: `https://ode.org/wiki/index.php/Manual`.

**Names that are NOT current API** — `dWorldStepFast1`, `dRayCreate`, `dCreateCCylinder`, `dInitODE` are removed/renamed; web tutorials still use them, and their modern replacements + cites are owned by `references/version-and-changelog.md` (the single deprecation-list owner).
Reminder: `dContactGeom` is the `.geom` member of `dContact`, not a separate object — pass `&contact[0].geom` with stride `sizeof(dContact)` (see `references/collision-and-contacts.md`).

> Note (from `web-best-practices.json` verify): several Gazebo knobs (kp, kd,
> `contact_max_correcting_vel`, friction-pyramid params) belong to Gazebo's ODE
> wrapper/SDF, not the raw ODE C API; the nearest raw equivalents are
> `surface.soft_erp`/`soft_cfm` and `dWorldSetContactMaxCorrectingVel`. Treat
> kp/kd as a derivation recipe, and tune the spring-damper formulas empirically.

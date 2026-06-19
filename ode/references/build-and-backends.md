# Build options and collision/OS backends

> Source of truth: CMakeLists.txt, configure.ac, INSTALL.txt, ode.pc.in, ode-config.in, build/premake4.lua, and the bundled OPCODE/GIMPACT/libccd/ou backends. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- ODE ships **two independent build systems with divergent defaults**: **CMake defaults to a SHARED library + double precision (on 64-bit)**, while **Autotools defaults to a STATIC library + single precision**. The compiled `dReal` differs, so the choice is load-bearing for your app (CMakeLists.txt:33,55-59; INSTALL.txt:69-83).
- Precision (`dSINGLE`/`dDOUBLE`) is a build-wide ABI decision. It must match between the ODE library, every bundled backend (libccd's `CCD_PRECISION`), and **your application** (configure.ac:131-134; CMakeLists.txt:134-140,142).
- Trimesh collision is a **build-time, mutually-exclusive backend choice**: **OPCODE** (default, stable, no scene-size limit) or **GIMPACT** (experimental, dynamic/deformable meshes, hard `[-1638,+1638]` scene limit). Pick exactly one (CMakeLists.txt:37-41; configure.ac:105-120).
- **libccd** is opt-in (OFF by default) and fills in robust convex/cylinder collider pairs ODE lacks native code for, gated per-pair by `dLIBCCD_*` defines (CMakeLists.txt:40,45-53).
- **OPCODE, GIMPACT, libccd, ou all live OUTSIDE `include/ode`** — they are bundled third-party internals with no installed headers and no API/ABI stability. User code touches **only** the public ODE C API in `include/ode/*.h` (CMakeLists.txt:318-383).
- Link your C program against **`-lstdc++` and `-lm`** in addition to ODE (ODE mixes C and C++); `ode.pc` already lists these under `Libs.private` (ode.pc.in:11-12).

## When to use

Read this when you must **configure, compile, or link** ODE — choosing precision, picking a trimesh backend, enabling libccd for cylinder/convex collisions, turning on threading (`ou`/TLS), deciding shared vs static, wiring `pkg-config`/`find_package(ODE)` into a consumer, or understanding what OPCODE/GIMPACT/libccd/ou are.

## Public API (C)

This module is a **build/configure surface**, not a runtime C API — the "symbols" are CMake options, Autotools flags, compile defines, and the consumer-integration files (`ode.pc`, `ode-config`, `find_package(ODE)`). The backends' own functions/types (`gimpact_init`, `ccdGJKIntersect`, `ccd_t`, `Opcode::Model`, `GIM_TRIMESH`) are **internal** and listed only so you do NOT call them.

### Full build-option matrix

| Option (CMake / Autotools) | Default | Effect | Source |
| --- | --- | --- | --- |
| `ODE_DOUBLE_PRECISION` (CMake) / `--enable-double-precision` (Autotools) | CMake: **ON** on 64-bit, **OFF** on 32-bit pointer targets. Autotools: **no** (single) | Selects `dDOUBLE` vs `dSINGLE` → size of `dReal`. CMake also sets `CCD_PRECISION` and `-DdIDEDOUBLE`/`-DCCD_IDEDOUBLE`; Windows renames lib `ode_double`/`ode_single`. Autotools substitutes `ODE_PRECISION` into `precision.h` and `ode.pc`. | CMakeLists.txt:55-59,134-140,530-534; configure.ac:123-135 |
| `BUILD_SHARED_LIBS` (CMake) / `--enable-shared` `--disable-shared` (Autotools/libtool) | CMake: **ON** (shared). Autotools: **disable-shared → STATIC** | ON → shared + `-DODE_DLL`; OFF → static + `-DODE_LIB`. Windows: drives `DEBUG_POSTFIX` (`d`/`sd`) and drawstuff `-DDS_DLL`/`-DDS_LIB`. Autotools default is static; pass `--enable-shared` for shared. | CMakeLists.txt:33,516-528,561-565; configure.ac:61 |
| `ODE_WITH_OPCODE` (CMake) / `--with-trimesh=opcode` (Autotools) | CMake: **ON**. Autotools: **opcode** (the `--with-trimesh` default) | Use OPCODE trimesh-trimesh collider; adds `-DdTRIMESH_ENABLED -DdTRIMESH_OPCODE` + OPCODE includes. Effective only when NOT `ODE_NO_TRIMESH`. | CMakeLists.txt:41,383-491,659-672; configure.ac:103-120 |
| `ODE_WITH_GIMPACT` (CMake) / `--with-trimesh=gimpact` (Autotools) | CMake: **OFF** (experimental). Autotools: not selected | Use GIMPACT instead of OPCODE; adds `-DdTRIMESH_ENABLED -DdTRIMESH_GIMPACT` + GIMPACT/include. Alternative to OPCODE, not additive. | CMakeLists.txt:39,318-349,597-600; configure.ac:105-120 |
| (no CMake option — set both OPCODE+GIMPACT OFF) `ODE_NO_TRIMESH` guard / `--with-trimesh=none` (Autotools) | trimesh **enabled** by default | Disable trimesh entirely → no `-DdTRIMESH_ENABLED`; `dGeomTriMesh*` are compiled out / no-op. `ODE_NO_TRIMESH` is only an internal guard, not a declared option(). | CMakeLists.txt:318,383,597,659,816; configure.ac:105-120 |
| `ODE_OLD_TRIMESH` (CMake) / `--enable-old-trimesh` (Autotools) | **OFF** / disabled | Legacy OPCODE trimesh-trimesh collider; adds `-DdTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER`. **OPCODE only** — ignored under GIMPACT. | CMakeLists.txt:37,662-664; configure.ac:247-257 |
| `ODE_WITH_LIBCCD` (CMake) / `--enable-libccd` (Autotools) | **OFF** / disabled | Use libccd for collider pairs ODE lacks. Adds `-DdLIBCCD_ENABLED` + `-DdLIBCCD_INTERNAL`/`-DdLIBCCD_SYSTEM` + `collision_libccd.cpp`. Autotools `--enable-libccd` enables **all libccd colliders except box-cylinder**. | CMakeLists.txt:40,351-381,602-657; configure.ac:425-440 |
| `ODE_WITH_LIBCCD_SYSTEM` (CMake) / `--with-libccd=[internal\|system]` (Autotools) | CMake: **OFF** (=internal). Autotools: **system** (falls back to internal if pkg-config `ccd` missing) | OFF → compile bundled `libccd/src/*.c` (`-DdLIBCCD_INTERNAL`); ON → `find_package(ccd REQUIRED)` + link `ccd::ccd` (`-DdLIBCCD_SYSTEM`). CMake dependent option (only when `ODE_WITH_LIBCCD`). | CMakeLists.txt:53; configure.ac:481-495 |
| `ODE_WITH_LIBCCD_BOX_CYL` (CMake) / `--with-box-cylinder=[default\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **default** (ODE's own; NOT enabled by `--enable-libccd`) | libccd box-cylinder (`-DdLIBCCD_BOX_CYL`). Must opt-in explicitly under Autotools via `--with-box-cylinder=libccd`. | CMakeLists.txt:45,626-628; configure.ac:416,446-448 |
| `ODE_WITH_LIBCCD_CAP_CYL` (CMake) / `--with-capsule-cylinder=[none\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **none** | libccd capsule-cylinder (`-DdLIBCCD_CAP_CYL`). | CMakeLists.txt:46,630-632; configure.ac:450-451 |
| `ODE_WITH_LIBCCD_CYL_CYL` (CMake) / `--with-cylinder-cylinder=[none\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **none** | libccd cylinder-cylinder (`-DdLIBCCD_CYL_CYL`). | CMakeLists.txt:47,634-636; configure.ac:443-444 |
| `ODE_WITH_LIBCCD_CONVEX_BOX` (CMake) / `--with-convex-box=[none\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **none** | libccd convex-box (`-DdLIBCCD_CONVEX_BOX`). | CMakeLists.txt:48,638-640; configure.ac:453-454 |
| `ODE_WITH_LIBCCD_CONVEX_CAP` (CMake) / `--with-convex-capsule=[none\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **none** | libccd convex-capsule (`-DdLIBCCD_CONVEX_CAP`). | CMakeLists.txt:49,642-644; configure.ac:456-457 |
| `ODE_WITH_LIBCCD_CONVEX_CONVEX` (CMake) / `--with-convex-convex=[default\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **default** (ODE's own) | libccd convex-convex (`-DdLIBCCD_CONVEX_CONVEX`). | CMakeLists.txt:50,646-648; configure.ac:465-466 |
| `ODE_WITH_LIBCCD_CONVEX_CYL` (CMake) / `--with-convex-cylinder=[none\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **none** | libccd convex-cylinder (`-DdLIBCCD_CONVEX_CYL`). | CMakeLists.txt:51,650-652; configure.ac:459-460 |
| `ODE_WITH_LIBCCD_CONVEX_SPHERE` (CMake) / `--with-convex-sphere=[default\|libccd]` | CMake: **ON** when LIBCCD on. Autotools: **default** | libccd convex-sphere (`-DdLIBCCD_CONVEX_SPHERE`). | CMakeLists.txt:52,654-656; configure.ac:462-463 |
| `ODE_WITH_OU` (CMake) / `--enable-ou` (Autotools) | **OFF** / no | Use TLS for global caches (enables threaded collision checks on separated/isolated spaces). Sets `_OU_FEATURE_SET_TLS`, adds `-DdATOMICS_ENABLED -DdTLS_ENABLED`, links threads. | CMakeLists.txt:42,110-116,674-682; configure.ac:320-324,336-357 |
| `ODE_NO_THREADING_INTF` (CMake) / `--disable-threading-intf` (Autotools) | **OFF** (interface enabled) | Disable threading interface (external threading impls cannot be assigned); adds `-DdTHREADING_INTF_DISABLED`. When enabled but not OU, feature set is `_OU_FEATURE_SET_ATOMICS` with `-DdATOMICS_ENABLED`. | CMakeLists.txt:36,112-113,593-595,676-678; configure.ac:315-319,399-413 |
| `ODE_NO_BUILTIN_THREADING_IMPL` (CMake) / `--disable-builtin-threading-impl` (Autotools) | **OFF** (impl included) | Disable the built-in multithreaded threading implementation; default defines `-DdBUILTIN_THREADING_IMPL_ENABLED`. Autotools impl only present when threading-intf is enabled. | CMakeLists.txt:35,589-591; configure.ac:401-409 |
| `ODE_WITH_DEMOS` (CMake) / `--disable-demos`, `--with-drawstuff=[X11\|Win32\|OSX\|none]` (Autotools) | CMake: **ON**. Autotools: demos **enabled**; drawstuff autodetected from host_os | Builds the demo apps **and** the DrawStuff library: `find_package(OpenGL REQUIRED)` + GLUT(APPLE)/X11(other)/winmm(WIN32). Trimesh demos only if NOT `ODE_NO_TRIMESH`. Autotools auto-disables demos if OpenGL/drawstuff missing. | CMakeLists.txt:38,63-67,730-856,816-826; configure.ac:138-169,223-234 |
| `ODE_WITH_TESTS` (CMake) — no Autotools toggle | CMake: **ON** | Builds the unit-test app (bundled UnitTest++): `tests` executable + `enable_testing()` + `add_test(tests ...)`. Autotools `tests/` Makefiles are generated unconditionally. | CMakeLists.txt:43,858-938; configure.ac:576-579 |
| `ODE_16BIT_INDICES` (CMake) / `--16bit-indices` (Premake only) | **OFF** (32-bit indices) | Use 16-bit trimesh indices; adds `-DdTRIMESH_16BIT_INDICES`. No Autotools flag. | CMakeLists.txt:34,585-587; build/premake4.lua:119-122 |
| `--disable-asserts` (Autotools) — CMake automatic | **enabled** (asserts on) | Disable debug error checking; adds `-DdNODEBUG -DNDEBUG`. CMake instead adds `dNODEBUG` automatically in every non-Debug config. | configure.ac:516-523; CMakeLists.txt:543 |
| `--disable-sse2` (Autotools) / `--no-sse2` (Premake) | **sse2=yes** | On 32-bit pentium adds `-mmmx -msse -msse2 -mfpmath=sse -mstackrealign`; ignored on x86_64. | configure.ac:276-292; build/premake4.lua:155-158 |
| `--enable-gprof` (Autotools) | **no** | Profiling with gprof: adds `-pg`, links `-lgmon` if present. | configure.ac:260-274 |
| `--disable-version-info` (Autotools) | version info **encoded** (`-version-info 8:0:0`) | Drops libtool version info (`-avoid-version`). SOVERSION major = 8. | configure.ac:29-40 |

### Backends survey — what each is, when selected, public-facing implication

| Backend | What it is | Selected by | Public-facing implication | Source |
| --- | --- | --- | --- | --- |
| **OPCODE** (OPtimized COllision DEtection, Pierre Terdiman) | ODE's **default** trimesh backend; hierarchical quantizable AABB-trees, ~7x smaller than RAPID, fast for mostly-static meshes; mesh-mesh + sphere/box/ray/OBB/plane queries. C++ `Opcode::` namespace. | `ODE_WITH_OPCODE=ON` (default) / `--with-trimesh=opcode`; define `-DdTRIMESH_OPCODE`. | Stable default for trimesh collision with **no scene-size limit**; optimized for static geometry (recomputes transformed verts each query for dynamic meshes). Internal — use only via `dGeomTriMesh*`/`dCollide`. | backends-survey.json (OPCODE/Opcode.h; CMakeLists.txt:41,660) |
| **GIMPACT** (Geometric tools, Francisco Leon) | **Experimental** alternative trimesh backend; AABB-tree for rigid meshes OR box-pruning for deformable, ~O(log n) updates on transform/vertex edit, lower memory for shared meshes. | `ODE_WITH_GIMPACT=ON` / `--with-trimesh=gimpact`; define `-DdTRIMESH_GIMPACT`. | Better for **moving/deformable** trimeshes, BUT scene coords are **hard-limited to [-1638,+1638]** (`MAX_AABB_SIZE`) — unusable for very large worlds (>~3276 unit span). Internal. | backends-survey.json (GIMPACT/include/GIMPACT/gimpact.h; gim_boxpruning.h:262; CMakeLists.txt:39,598) |
| **libccd** (Daniel Fiser) | Convex-shape collision lib: GJK (intersect/distance), EPA (penetration depth), MPR/XenoCollide. Each geom plugs in via a "support function". | `ODE_WITH_LIBCCD=ON` (default OFF) + per-pair sub-options; defines `-DdLIBCCD_ENABLED` + `-DdLIBCCD_*`. Bundled (internal) or system (`find_package(ccd)`). | Enable it to get **reliable cylinder/convex collisions** (box-cyl, cap-cyl, cyl-cyl, convex-box/cap/convex/cyl/sphere); `CCD_SINGLE`/`CCD_DOUBLE` must track ODE precision. Internal. | backends-survey.json (libccd/src/ccd/ccd.h; CMakeLists.txt:40,45-53,609) |
| **ou** (ODER's Utilities, Oleh Derevenko) | **NOT a collision backend** — ODE's portability/concurrency lib: cross-platform atomics (interlocked ops), thread-local storage, assertion macros, flag/enum templates. Compiled under C++ namespace `odeou`. | `ODE_WITH_OU` + threading interface; `_OU_FEATURE_SET` tier BASICS/ATOMICS/TLS; defines `-DdATOMICS_ENABLED`/`-DdTLS_ENABLED`. | Building with `ODE_WITH_OU` (TLS) lets you run collision checks on **separate `dSpace`s from different threads** safely; without it those global caches are not thread-isolated. Internal. | backends-survey.json (ou/README.TXT; CMakeLists.txt:42,110-128,117,674-678) |

### Consumer integration files

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `ode.pc` (pkg-config) | `Libs: -L${libdir} -lode` / `Libs.private: -lstdc++ -lm` / `Cflags: -I${includedir}` / `precision=@ODE_PRECISION@` | ode.pc.in:1-13 | Preferred consume path: `pkg-config --cflags --libs ode`. Carries the precision so consumers match the lib. |
| `ode-config` (legacy script) | `--prefix[=DIR]` `--exec-prefix[=DIR]` `--version` `--cflags` `--libs` | ode-config.in:1-54 | Backward-compat alternative to pkg-config; installed alongside `ode.pc`. |
| `find_package(ODE)` / `ODE::ODE` | `find_package(ODE)` + `target_link_libraries(app ODE::ODE)` | ode-config.cmake.in:1-73; CMakeLists.txt:21,685-726,940-967 | CMake package config exposing `ODE_VERSION*`, all `ODE_*` option values, `ODE_INCLUDE_DIRS/LIBRARIES`, the `ODE::ODE` imported target; `find_dependency(ccd)` if libccd is system. |
| Project version | `0.16.0` (API); SOVERSION `8.0.0`, SONAME major `8` | CMakeLists.txt:23-31,512-513; configure.ac:2-3 | Library version reported by `ode.pc` (`Version:`) and `ode-config --version`. |

### Internal backend symbols — DO NOT call from user code

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `gimpact_init` | `void gimpact_init();` | GIMPACT/include/GIMPACT/gimpact.h:41 | INTERNAL — initializes GIMPACT; called by ODE's GIMPACT wrapper, not user-facing. |
| `gimpact_terminate` | `void gimpact_terminate();` | GIMPACT/include/GIMPACT/gimpact.h:43 | INTERNAL — cleans up GIMPACT; paired with `gimpact_init()`. |
| `ccdGJKIntersect` | `int ccdGJKIntersect(const void *obj1, const void *obj2, const ccd_t *ccd);` | libccd/src/ccd/ccd.h | INTERNAL — boolean convex-convex GJK test ODE calls when `-DdLIBCCD_ENABLED`. Not a public ODE symbol. |
| `ccd_t` (`struct _ccd_t`) | (libccd config struct: support1/2, first_dir, center, iteration limits, epsilon) | libccd/src/ccd/ccd.h:52 | INTERNAL — populated per geom pair; never exposed through `include/ode`. |
| `ccd_support_fn` / `ccd_first_dir_fn` / `ccd_center_fn` | (callback typedefs) | libccd/src/ccd/ccd.h:34 | INTERNAL — the "support function" returning the furthest point of a convex object in a direction; how ODE plugs each geom into GJK/EPA/MPR. |
| `Opcode::Model` / `OPCODE_Model` | (C++ class wrapping an AABB-tree over a triangle mesh) | OPCODE/Opcode.h | INTERNAL — built per `dTriMeshData` by ODE's OPCODE backend; C++ `Opcode` namespace, not the C API. [VERIFY] exact class name not grep-pinned. |
| `GIM_TRIMESH` | (GIMPACT triangle-mesh container) | GIMPACT/include/GIMPACT/gim_trimesh.h | INTERNAL — supports linear-transform updates and direct vertex edits; ODE's GIMPACT backend wraps it. [VERIFY] exact struct/typedef name not grep-pinned. |

## Key rules

- Precision must match between the ODE library, the bundled backends, **and your application**: build with `dSINGLE` or `dDOUBLE` and define the SAME macro when compiling your code (or just consume the installed headers / `ode.pc` which carry `precision=@ODE_PRECISION@`); `dReal` is float vs double and a mismatch corrupts every struct crossing the ABI (configure.ac:123-135; CMakeLists.txt:134-140).
- **Autotools default = STATIC library, SINGLE precision, OPCODE trimesh, debug symbols/asserts enabled**; pass `--enable-shared` for shared and `--enable-double-precision` for double (INSTALL.txt:69-83).
- **CMake default = SHARED library (`BUILD_SHARED_LIBS` ON), OPCODE ON, demos ON, tests ON, libccd OFF, GIMPACT OFF**, and `ODE_DOUBLE_PRECISION` ON on 64-bit / OFF on 32-bit — these differ from Autotools (static + single) (CMakeLists.txt:33-59).
- To disable trimesh under CMake set BOTH `ODE_WITH_OPCODE=OFF` and `ODE_WITH_GIMPACT=OFF` (there is no `ODE_NO_TRIMESH` option to declare); under Autotools use `--with-trimesh=none` (CMakeLists.txt:39-41,318,383).
- OPCODE and GIMPACT are **alternative** trimesh-trimesh colliders, not additive — Autotools `--with-trimesh` picks exactly one (opcode XOR gimpact) (configure.ac:105-120).
- If NEITHER trimesh backend is compiled, `dGeomTriMesh*` functions are compiled out / silently no-op and trimesh geoms collide with nothing (OPCODE/README-ODE.txt:6).
- The per-shape libccd colliders (`ODE_WITH_LIBCCD_*`) only take effect when `ODE_WITH_LIBCCD` is ON; CMake enforces this via `cmake_dependent_option` (they force OFF and vanish from the cache otherwise) (CMakeLists.txt:45-53).
- Autotools `--enable-libccd` enables all libccd colliders **EXCEPT box-cylinder**; add `--with-box-cylinder=libccd` explicitly for that pair (configure.ac:427-448).
- libccd precision must match ODE precision: CMake sets `CCD_PRECISION = CCD_DOUBLE/CCD_SINGLE` in tandem with `ODE_DOUBLE_PRECISION`; mismatching is a configuration error (CMakeLists.txt:142).
- GIMPACT restricts usable scene coordinates to `[-1638,+1638]` (`MAX_AABB_SIZE`) because of its 2D box-pruning sort; OPCODE has no such limit — choose OPCODE for large worlds (GIMPACT/include/GIMPACT/gim_boxpruning.h:262).
- All four backends (OPCODE/GIMPACT/libccd/ou) are bundled internals OUTSIDE `include/ode`; never include or call their headers — interact only through `include/ode/*.h` (CMakeLists.txt:318-383).
- `ODE_OLD_TRIMESH` (`-DdTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER`) only applies to the OPCODE backend (CMakeLists.txt:662-664).
- For C programs you must also link `-lstdc++` and `-lm`; `ode.pc` lists these under `Libs.private` (ode.pc.in:11-12).
- Set BOTH `CFLAGS` and `CXXFLAGS` for optimization flags under Autotools because ODE mixes C and C++ source (INSTALL.txt:85-94).
- When building from a VCS/Subversion checkout run `./bootstrap` (needs autoconf/automake/libtool/pkg-config) before `./configure`; released tarballs already ship the generated `configure` (INSTALL.txt:41-57; bootstrap:1-30).
- Prefer pkg-config (`pkg-config --cflags --libs ode`) over the legacy `ode-config` script; both are installed by `make install` (INSTALL.txt:101-103; ode-config.in:1-54).
- On Windows the CMake library `OUTPUT_NAME` is `ode_single`/`ode_double` (precision-suffixed) with `s`/`sd` static postfixes — consumers should use `find_package(ODE)` + `ODE::ODE`, not a literal `-lode` (CMakeLists.txt:516-535).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `./configure --enabled-shared` | `./configure --enable-shared` | INSTALL.txt:82 contains a typo; the libtool/autoconf flag is `--enable-shared` and the default is a static library because `LT_INIT` uses `disable-shared` (configure.ac:61). |
| Compile the ODE lib in double precision but the app at default single (no `-DdDOUBLE`). | Build the lib with `--enable-double-precision`/`ODE_DOUBLE_PRECISION=ON` and define the matching `dDOUBLE` when compiling your code, or consume the installed headers/`ode.pc` carrying `precision=@ODE_PRECISION@`. | `dReal` is float vs double; an ABI/size mismatch silently corrupts every `dBodyID`/`dVector3`/struct across the boundary (configure.ac:123-135). |
| Set only `CFLAGS` (e.g. `-msse`) under Autotools. | Set both: `./configure CFLAGS="-msse" CXXFLAGS="-msse"`. | ODE is a mix of C and C++ files; one-language flags leave half the codebase unoptimized or with mismatched codegen (INSTALL.txt:93-94). |
| Treat `--with-trimesh=opcode` and `=gimpact` as additive / both applied. | Pick exactly one: `--with-trimesh=opcode` (default), `=gimpact`, or `=none`. | configure.ac:105-120 sets opcode XOR gimpact — they are alternative trimesh-trimesh colliders, not stackable. |
| Set `ODE_WITH_LIBCCD_CYL_CYL=ON` in CMake while leaving `ODE_WITH_LIBCCD=OFF`. | Enable `ODE_WITH_LIBCCD=ON` first; the per-shape options are `cmake_dependent_option`. | The dependent options have no effect (and are removed from the cache) unless the parent `ODE_WITH_LIBCCD` is ON (CMakeLists.txt:45-53). |
| Run `./configure` directly on a git/Subversion checkout. | Run `./bootstrap` first, then `./configure`. | A VCS checkout lacks the generated `configure`/`Makefile.in`; only released tarballs ship them (INSTALL.txt:41-57). |
| Hard-code `-lode` from CMake on Windows. | Use `find_package(ODE)` and link the `ODE::ODE` target. | On Windows the CMake output name is `ode_single`/`ode_double` with debug/static postfixes, so a literal `-lode` will not resolve (CMakeLists.txt:516-535). |
| `#include "Opcode.h"` / `<ccd/ccd.h>` / `<GIMPACT/gimpact.h>` and call their functions. | Include only public ODE headers (e.g. `include/ode/collision_trimesh.h`) and use `dGeomTriMesh*`/`dCollide`. | Those directories sit outside `include/ode`, are compiled into libode as private internals, are not installed, and have no API/ABI stability guarantee (CMakeLists.txt:318-383). |
| Place GIMPACT-built trimesh geometry beyond ±1638 units. | Keep the simulation within ±1638 units under GIMPACT, or build with OPCODE (the default), which has no such limit. | GIMPACT's box-pruning sort (`MAX_AABB_SIZE 1638.0f`) silently loses precision / produces wrong overlaps outside that range (GIMPACT/include/GIMPACT/gim_boxpruning.h:262). |
| Compile ODE in double precision but libccd in single (or vice versa). | Let the build set `CCD_PRECISION` from `ODE_DOUBLE_PRECISION` (CMake does this automatically); don't override one without the other. | Precision mismatch between `dReal` and libccd's `ccd_real_t` corrupts collision results at the ABI boundary (CMakeLists.txt:142). |
| Assume convex-convex / cylinder-cylinder / box-cylinder always work out of the box. | Build with `ODE_WITH_LIBCCD` (+ the relevant `dLIBCCD_*` pair flags) for robust colliders. | libccd is OFF by default; several pairs (e.g. cylinder-cylinder) are only well-handled when libccd is compiled in (CMakeLists.txt:45-53). |

## Named constants

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dSINGLE` / `dDOUBLE` | precision macros selected via `ODE_PRECISION` | configure.ac:131-134 | Exactly one must be defined consistently for the lib AND every consumer; sets the size of `dReal`. |
| `dTRIMESH_ENABLED` | define present when OPCODE or GIMPACT built | CMakeLists.txt:598 | When absent, `dTriMesh*` functions are stubbed / no-op. |
| `dTRIMESH_OPCODE` | define for OPCODE backend | CMakeLists.txt:660 | Selects OPCODE as the trimesh-trimesh collider (the default). Paired with `dTRIMESH_ENABLED`. |
| `dTRIMESH_GIMPACT` | define for GIMPACT backend | CMakeLists.txt:598 | Selects GIMPACT instead of OPCODE; mutually exclusive with `dTRIMESH_OPCODE`. |
| `dTRIMESH_16BIT_INDICES` | from `ODE_16BIT_INDICES`/`--16bit-indices` | CMakeLists.txt:585-587 | Use 16-bit trimesh indices (default 32-bit). |
| `dTRIMESH_OPCODE_USE_OLD_TRIMESH_TRIMESH_COLLIDER` | from `ODE_OLD_TRIMESH`/`--enable-old-trimesh` | CMakeLists.txt:662-664 | Legacy OPCODE trimesh-trimesh collider; OPCODE only. |
| `dLIBCCD_ENABLED` / `dLIBCCD_INTERNAL` / `dLIBCCD_SYSTEM` | from `ODE_WITH_LIBCCD` / `ODE_WITH_LIBCCD_SYSTEM` | CMakeLists.txt:609 | Enable libccd; INTERNAL = bundled `libccd/src`, SYSTEM = `find_package(ccd)`. |
| `dLIBCCD_BOX_CYL`, `dLIBCCD_CAP_CYL`, `dLIBCCD_CYL_CYL`, `dLIBCCD_CONVEX_BOX`, `dLIBCCD_CONVEX_CAP`, `dLIBCCD_CONVEX_CONVEX`, `dLIBCCD_CONVEX_CYL`, `dLIBCCD_CONVEX_SPHERE` | per-pair defines | CMakeLists.txt:609,626-656 | Choose which collider pairs libccd handles (one per shape pair). |
| `dATOMICS_ENABLED` / `dTLS_ENABLED` | ou-related | CMakeLists.txt:674-678 | `dATOMICS_ENABLED` when threading interface on; `dTLS_ENABLED` additionally when `ODE_WITH_OU` on (TLS global caches → cross-thread space collision). |
| `dBUILTIN_THREADING_IMPL_ENABLED` | built-in threading impl present | CMakeLists.txt:589-591 | Defined unless `ODE_NO_BUILTIN_THREADING_IMPL`. |
| `dTHREADING_INTF_DISABLED` | from `ODE_NO_THREADING_INTF` | CMakeLists.txt:593-595 | Disables the threading interface (external impls cannot be assigned). |
| `dNODEBUG` / `NDEBUG` | from `--disable-asserts` (Autotools) / non-Debug config (CMake) | configure.ac:516-523; CMakeLists.txt:543 | Disable debug error checking. CMake adds `dNODEBUG` automatically in every non-Debug config. |
| `ODE_DLL` / `ODE_LIB` | from `BUILD_SHARED_LIBS` | CMakeLists.txt:561-565 | `-DODE_DLL` for shared, `-DODE_LIB` for static (drives symbol import/export). |
| `_OU_NAMESPACE` | `odeou` | CMakeLists.txt:117 | ou is compiled under C++ namespace `odeou` so multiple OU copies can link into one binary. |
| `MAX_AABB_SIZE` | `1638.0f` | GIMPACT/include/GIMPACT/gim_boxpruning.h:262 | INTERNAL GIMPACT limit: usable scene coords `[-1638,+1638]` (~3276x3276 max room). Geometry outside may mis-collide. |
| `ODE_VERSION` / project VERSION | `0.16.0` (API); SOVERSION `8.0.0` | CMakeLists.txt:28-31,23-26; configure.ac:2-3 | Library API version; reported by `ode.pc` and `ode-config --version`. |
| libtool `-version-info` | `8:0:0` (CURRENT:REVISION:AGE) | configure.ac:29-40 | SONAME major 8 (matches `SOVERSION_MAJOR 8`); `--disable-version-info` switches to `-avoid-version`. |

## Things to never invent

- Do not invent CMake options: the complete `option()`/`cmake_dependent_option()` set is on CMakeLists.txt:33-59 (`BUILD_SHARED_LIBS`, `ODE_16BIT_INDICES`, `ODE_NO_BUILTIN_THREADING_IMPL`, `ODE_NO_THREADING_INTF`, `ODE_OLD_TRIMESH`, `ODE_WITH_DEMOS`, `ODE_WITH_GIMPACT`, `ODE_WITH_LIBCCD`, `ODE_WITH_OPCODE`, `ODE_WITH_OU`, `ODE_WITH_TESTS`, `ODE_DOUBLE_PRECISION`, and the nine `ODE_WITH_LIBCCD_*` dependent options). There is NO `ODE_WITH_TRIMESH` option and NO declared `ODE_NO_TRIMESH` option.
- Do not invent Autotools flags: the full set is in configure.ac (`--disable-version-info`, `--with-trimesh`, `--enable-double-precision`, `--with-drawstuff`, `--disable-demos`, `--enable-old-trimesh`, `--enable-gprof`, `--disable-sse2`, `--disable-threading-intf`, `--enable-ou`, `--disable-builtin-threading-impl`, `--enable-libccd`, `--with-{cylinder-cylinder,box-cylinder,capsule-cylinder,convex-box,convex-capsule,convex-cylinder,convex-sphere,convex-convex}`, `--with-libccd`, `--disable-asserts`).
- Do not claim Autotools has a `--with-opcode` or `--with-gimpact` flag; the only trimesh selector is `--with-trimesh=[opcode|gimpact|none]` (configure.ac:105-108).
- Do not claim CMake has a `--enable-double-precision` flag or Autotools has an `ODE_DOUBLE_PRECISION` variable — keep the two systems' option names distinct.
- Do not state the same defaults for both build systems: Autotools defaults to static+single (INSTALL.txt:69), CMake defaults to shared + (double on 64-bit) (CMakeLists.txt:33,55-59).
- Do not invent `ode-config` flags beyond `--prefix`, `--exec-prefix`, `--version`, `--cflags`, `--libs` (ode-config.in:7-8).
- `no-dif` and `all-collis-libs` are Premake-only triggers (build/premake4.lua:84-87,94-97); do not present them as CMake or Autotools options.
- Do NOT present any backend symbol (`Opcode::*`, `ccdGJK*`, `gimpact_*`, `odeou::*`) as part of the public ODE API — they are internal and outside `include/ode`.
- Do NOT claim a specific GIMPACT scene limit other than ±1638 (`MAX_AABB_SIZE`) — it is the value in gim_boxpruning.h:262.
- Do NOT invent libccd collider-pair flags beyond the eight present: `dLIBCCD_BOX_CYL`, `dLIBCCD_CAP_CYL`, `dLIBCCD_CYL_CYL`, `dLIBCCD_CONVEX_BOX`, `dLIBCCD_CONVEX_CAP`, `dLIBCCD_CONVEX_CONVEX`, `dLIBCCD_CONVEX_CYL`, `dLIBCCD_CONVEX_SPHERE`.
- Do NOT state OPCODE and GIMPACT can both be active in one build — the trimesh backend is mutually exclusive (one `-DdTRIMESH_OPCODE` or `-DdTRIMESH_GIMPACT`).
- Do NOT describe ou as a collision library — it is a utilities/atomics/TLS library.
- Do NOT assert ABI/header stability for any backend; they are private build internals with no installed headers.

## Build recipes

```bash
# Autotools, released tarball (DEFAULT: static, single precision, OPCODE)  [INSTALL.txt:59-103]
tar xvfz ode-x.y.tar.gz && cd ode-x.y
./configure                            # autodetect
#   --enable-double-precision          double-precision math
#   --with-trimesh=none|opcode|gimpact trimesh backend (default opcode)
#   --enable-shared                    build a shared library
# Optimized: set BOTH CFLAGS and CXXFLAGS
# ./configure CFLAGS="-msse -march=atlon-xp" CXXFLAGS="-msse -march=atlon-xp"
make && make install                   # also installs ode.pc + ode-config
```

```bash
# Autotools, VCS (Subversion) checkout — bootstrap first  [INSTALL.txt:41-57; bootstrap:1-30]
./bootstrap                            # needs autoconf, automake, libtool, pkg-config
./configure && make && make install
```

```bash
# CMake out-of-source (recommended)  [INSTALL.txt:144-169]
mkdir ../ode-build && cd ../ode-build
cmake ../ode-src
#   -DODE_DOUBLE_PRECISION=ON  -DBUILD_SHARED_LIBS=OFF
#   -DODE_WITH_GIMPACT=ON -DODE_WITH_OPCODE=OFF   # GIMPACT instead of OPCODE
#   -DODE_WITH_LIBCCD=ON                          # enable libccd collider pairs
#   -DODE_WITH_OU=ON                              # TLS global caches (threaded spaces)
# Pick a generator with -G"<generator>"
```

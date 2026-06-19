# Building & running an ODE program (C++ on a real machine)

> The toolchain reality, not just the API. Every fact here was confirmed by building real programs
> against an installed ODE (Homebrew 0.16.6) — see `assets/Makefile.example`, `assets/headless_example.c`.
> Read it before you write a build command.

## 1. Find the installed ODE — don't rebuild it

ODE is almost always already installed; use it, don't compile the library from source. Detect it:

```sh
ode-config --version          # e.g. 0.16.6
ode-config --cflags           # -I/opt/homebrew/Cellar/ode/0.16.6/include
ode-config --libs             # -L/opt/homebrew/Cellar/ode/0.16.6/lib -lode
# or, equivalently:
pkg-config --modversion ode
pkg-config --cflags --libs ode
# install if missing (macOS):
brew install ode
```

`ode-config` ships with ODE (generated from `ode-config.in`). **Always take cflags/libs from it** — that
guarantees the include path *and the precision* match the installed library. (On this machine the lib is
under `/opt/homebrew/Cellar/ode/0.16.6/`.)

## 2. libode is a C++ library — link with the `c++` driver

This is the single most common build failure and it is **not** an API problem. `libode` is built from C++,
so it depends on the C++ runtime (`libc++`/`libstdc++`). If you compile a `.c` program with `cc`/`gcc` you
get a successful compile but a **link failure on C++ runtime symbols**.

Fix: drive the build with `c++` (clang++/g++). For a C source, add `-x c` so the *source* is still compiled
as C while the *driver* links the C++ runtime:

```sh
# C source (compiled as C, linked with the C++ runtime):
c++ -O2 -Wall $(ode-config --cflags) -x c  myprog.c   $(ode-config --libs) -o myprog

# C++ source (the path you want — same C ODE API, just a .cpp):
c++ -O2 -Wall -std=c++17 $(ode-config --cflags) myprog.cpp $(ode-config --libs) -o myprog
```

(A harmless `ld: warning: ignoring duplicate libraries '-lc++'` can appear if you also pass `-lstdc++`;
drop the explicit `-lstdc++`.) The canonical Makefile is `assets/Makefile.example`.

> **zsh ad-hoc gotcha:** in an interactive **zsh** one-liner, an *unquoted* `$(ode-config --libs)` is **not**
> word-split — it's passed as a single `"-L… -lode"` argument and the compile fails. `make`, `sh`, and
> `bash` split it as expected. In zsh, prefer `make`, or force the split with `c++ ... ${=$(ode-config --libs)}`.

## 3. Precision: define nothing, assert at runtime

`dReal` is `float` under `dSINGLE` and `double` under `dDOUBLE`, and `include/ode/common.h:64` is a hard
`#error` if neither is set. **Do not hand-define either flag.** `#include <ode/ode.h>` pulls in
`<ode/precision.h>`, which already matches the installed library (Homebrew's build is **double**). Defining
the *wrong* one silently misreads every `dReal`/`dVector3` — the #1 ODE integration bug. Verify at runtime:

```c
#include <ode/ode.h>
if (!dCheckConfiguration("ODE_double_precision"))   /* common.h:573, dGetConfiguration at :563 */
    { fprintf(stderr, "ODE precision mismatch\n"); return 1; }
```

If you build for double, the **drawstuff** draw calls need the `*D` variants — route them:
```c
#ifdef dDOUBLE
#  define dsDrawBox dsDrawBoxD   /* + dsDrawSphereD, dsDrawCapsuleD, dsDrawCylinderD, dsDrawLineD */
#endif
```

## 4. Using ODE *from C++* (what "C++ ODE" actually means)

In practice — confirmed across every real program built here — **C++ ODE means the plain C API written in a
`.cpp` and built with the `c++` driver.** The C API is the dominant, fully-supported path; you do not need
the wrapper. Good C++ hygiene over the C handles:

- Wrap handles in RAII if you like: a small struct whose destructor calls `dBodyDestroy`/`dGeomDestroy`.
  Keep collections as `std::vector<dBodyID>` etc.
- **The near-callback must be a plain or `static` function (or a non-capturing lambda decayed to a function
  pointer) — `dSpaceCollide` takes a C function pointer, so a *capturing* lambda will not convert.** Pass
  per-call state through the `void *data` argument instead.
- `extern "C"` is already handled: `include/ode/ode.h` guards the C API for C++ (`include/ode/README`).
- The optional `odecpp.h` wrapper classes (`dWorld`, `dBody`, `dHingeJoint`, …) exist — see
  `references/cpp-api.md` — but were never needed in any real build. Treat them as optional sugar.

## 5. Building inside the ODE *source* repo (only if you must)

If you are adding a demo to the ODE source tree (not a standalone project):
- Configure with CMake **and pass `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`** on modern CMake (4.x), or
  configure fails with *"Compatibility with CMake < 3.5 has been removed."*
- Register a new demo in the explicit demo list (`ALL_DEMO_SRCS`/`ODE_WITH_DEMOS` in `CMakeLists.txt`) —
  it is **not** glob-based — and use **tab** indentation. Trimesh demos sit behind a trimesh-enabled gate.
- Single-target build + run: `cmake --build build --target <demo> && ./build/<demo>`.

## 6. Version note

This skill's `file:line` citations are against the **0.16** source repo in this workspace; the installed
library is **0.16.6**. Same public API — line numbers in the installed headers may differ by a few.

**Verify a symbol against YOUR installed ODE** (turns `ode-config` into a grep path — do this whenever an
exact signature or line matters):
```sh
ODE_INC=$(ode-config --cflags | sed 's/-I//')    # or: pkg-config --variable=includedir ode
grep -rn 'dJointCreateHinge' "$ODE_INC/ode/"     # confirm the exact signature + line
```
Rule: **cite by symbol + nearest signature; treat this skill's repo line numbers as approximate.** (E.g.
`dJointCreateHinge` is `objects.h:1701` in the 0.16 repo but `objects.h:1664` in the installed 0.16.6.)

## Pre-flight (build)
- [ ] flags from `ode-config`/`pkg-config`, not hand-written `-I`/`-L`
- [ ] linked with the `c++` driver (`-x c` if the source is `.c`)
- [ ] no `-DdSINGLE`/`-DdDOUBLE`; runtime `dCheckConfiguration` guard present
- [ ] `#include <ode/ode.h>` only — never an internal `ode/src` header

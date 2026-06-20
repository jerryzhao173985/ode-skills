# Rendering & headless verification

> ODE simulates; it does **not** draw. The bundled **drawstuff** is an optional debug renderer, *not* the
> physics API. This page covers the two things every real build hit: (1) how to **verify a simulation with
> no window** (the pattern agents kept re-inventing — make it your default), and (2) how to render on a
> modern machine when you actually want pixels. Working templates: `assets/headless_example.c`,
> `assets/buggy_ramp.cpp`, `assets/glut_backend.cpp`, `assets/Makefile.example`.

## 1. Default to headless + a self-checking harness

A coding agent (or CI) **cannot assert on a window**. So make the physics observable as numbers and have
the program grade itself. This is not a fallback — it is the *better* way to know a sim is correct.

Pattern (the same physics builds with or without a renderer via `-DHEADLESS`):

```c
/* compile the SAME simulation two ways:
   c++ ... prog.cpp $(ode-config --libs) -o prog              # (with a renderer, if you have one)
   c++ ... -DHEADLESS prog.cpp $(ode-config --libs) -o prog_h # headless self-test                  */
static void step_world(void) {
    dSpaceCollide(space, 0, &nearCallback);
    dWorldQuickStep(world, DT);          /* fixed DT — never vary it */
    dJointGroupEmpty(contactgroup);
}
#ifdef HEADLESS
int main(void) {
    setup();
    for (int i = 0; i < STEPS; i++) {
        step_world();
        if (i % PRINT_EVERY == 0) report_state();   /* energy, zmin, vmax, joint separation */
    }
    int ok = check_invariants();         /* PE→KE then dissipates; |joint gap| tiny at rest; no NaN */
    printf("RESULT: %s\n", ok ? "PASS" : "FAIL");
    teardown();
    return ok ? 0 : 1;
}
#endif
```

**What to print/assert** (the quantities that make a run objectively verifiable):
- **Total energy** `E = KE + PE` — must fall then hold. A *growing* E means instability (bad ERP/CFM,
  too-large `DT`, or a precision mismatch).
- **Resting joint separation** — the gap between a joint's two anchor read-backs; should be a tiny fraction
  of a mm once motion stops. The real constraint-integrity test for jointed bodies.
- **`zmin` / `vmax`** — lowest body and fastest body; catch tunneling (a body below the floor) and blow-ups.
- **No `NaN`** in any position — the instant explosion check.

`assets/headless_example.c` (falling stack) and `assets/buggy_ramp.cpp` (`-DHEADLESS` prints a buggy
trajectory `[PASS] drove forward / climbed ramp / stayed stable`) are complete worked harnesses.

## 2. The drawstuff API (when you do render)

drawstuff is a throwaway immediate-mode OpenGL renderer (`include/drawstuff/drawstuff.h`). The skeleton:

```c
#include <drawstuff/drawstuff.h>
static void simLoop(int pause){ if(!pause) step_world(); /* dsSetColor + dsDrawBox/Sphere/Capsule/Cylinder */ }
int main(int argc,char**argv){
  dsFunctions fn; fn.version=DS_VERSION; fn.start=&start; fn.step=&simLoop;
  fn.command=&command; fn.stop=0; fn.path_to_textures=DRAWSTUFF_TEXTURE_PATH;
  setup();
  dsSimulationLoop(argc,argv,640,480,&fn);   /* owns the window + camera; calls fn.step each frame */
  teardown();
}
```
- `dsSetViewpoint`'s `hpr` is **degrees** (the only degrees in the stack — the physics API is radians).
- Under a **double** build, use the `*D` draw variants (see `building-and-running.md` §3).
- Pass per-frame geometry via `dBodyGetPosition`/`dBodyGetRotation` straight into `dsDrawBox(...)`.

## 3. Rendering on modern macOS — the gotcha

**Homebrew's `libode` ships the physics library only — no drawstuff** (no header, no lib, no textures).
ODE's stock macOS backend `drawstuff/src/osx.cpp` is actually **GLUT-based** — despite a stale `// Carbon+AGL`
comment at the top (`osx.cpp:23`), the code includes `<OpenGL/gl.h>` + `<GLUT/glut.h>` and uses GLUT throughout
(`osx.cpp:44-45`; verify: `grep -n include drawstuff/src/osx.cpp`). GLUT is *deprecated* on modern macOS (since
10.9) but still ships and links, with deprecation warnings. So to render on a Mac you must:

1. **Vendor drawstuff from the ODE source** (`third_party/drawstuff/`): the *core* `drawstuff.cpp` is
   platform-agnostic and used **unmodified** — it owns camera/lighting/textures and the `dsDraw*` calls.
2. **Supply a window/GL-context backend** implementing the contract in `drawstuff/src/internal.h`:
   `dsPlatformSimLoop`, `dsError`, `dsDebug`, `dsPrint`, `dsStop`, `dsElapsedTime`. Each frame it calls the
   core's `dsDrawFrame()` (which calls your `fn.step`) and swaps buffers; keyboard/mouse drive the command
   callback + core camera (`dsMotion`).
3. Use **GLUT** (present on macOS by default; no XQuartz): link `-framework GLUT -framework OpenGL
   -framework Cocoa`, and set `config.h` to a single `#define HAVE_APPLE_OPENGL_FRAMEWORK`.

`assets/glut_backend.cpp` is a complete, working ~280-line GLUT backend you can drop in; `assets/Makefile.example`
shows the exact compile (vendored `drawstuff.o` + `glut_backend.o` + your `.cpp` + frameworks). **Start from the asset.**
(Since stock `osx.cpp` is itself GLUT, vendoring it directly is also an option — though, like the whole rendering
path here, that's reasoned from the source, not field-tested on this machine; the asset is the more minimal route.)

## When to render vs not
- **Building, testing, CI, agent-driven work →** headless + self-check (§1). Always your first move.
- **Demoing / debugging visually / a human in the loop →** drawstuff (§2), with the macOS backend (§3) if
  on a Mac. Linux/X11 can use ODE's stock `x11.cpp` backend instead.

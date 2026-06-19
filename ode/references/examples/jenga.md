# Example: Jenga Tower Knock-Over (stand-then-topple + energy self-check)

Builds a classic 54-block Jenga tower (18 layers x 3 blocks, alternating 90 degrees),
lets it settle under gravity, hurls a heavy "wrecking ball" sphere into its lower-middle,
and runs through impact until everything comes to rest. It is a tested, headless,
self-checking standalone C++ program: it prints nothing you have to look at to judge it —
it runs a battery of physical self-checks and returns its verdict as the process exit
code (`0` = all checks passed). The shipped, field-tested source is `assets/jenga.cpp`.

## What it demonstrates

- **The full ODE pipeline for stable stacking**: world + hash space + bodies + box geoms
  + a static ground plane, a `dSpaceCollide` -> `dCollide` -> contact-joint near-callback,
  and `dWorldQuickStep` with **raised SOR iterations** (80 vs. the default 20) so a deep
  contact island actually converges (`assets/jenga.cpp:322`).
- **Resting-contact tuning that keeps a 54-box stack from exploding or jittering**: soft
  CFM/ERP contacts, a contact surface layer, capped correcting velocity, light
  linear/angular damping, and **auto-disable** so settled blocks sleep
  (`assets/jenga.cpp:319-333`).
- **Rolling + spinning friction** (`dContactRolling`, `surface.rho/rho2/rhoN`) so the ball
  comes to rest instead of rolling forever and preventing the system from ever settling
  (`assets/jenga.cpp:121-128`).
- **Energy computed from first principles** — ODE has no energy function — including the
  correct rotation of angular velocity into the *body* frame before applying the body-frame
  inertia tensor `dMass.I` (`assets/jenga.cpp:142-167`).
- **A two-phase verification harness that can actually fail**: phase 1 proves the tower
  *stood*; phase 3 proves it *toppled*; an energy-conservation check uses a *fixed launch
  baseline* so a buggy or runaway solver would trip it. The whole point: a passing check
  that cannot fail is worthless.
- **Defensive C++ for a C API**: a `static_assert` precision guard, a runtime
  `dCheckConfiguration` guard, file-scope globals because the collide callback is a plain C
  function pointer that cannot capture, and a `Check` table that pre-formats its detail
  string (no runtime format-string pitfalls).

## Build & run

Double precision is ODE's Homebrew default, so define **no** precision macro — let
`ode-config` supply the flags:

```sh
c++ -O2 -std=c++17 $(ode-config --cflags) jenga.cpp $(ode-config --libs) -o jenga
./jenga; echo "exit=$?"
```

It is fully headless (no drawstuff, no window, no input). The exit code *is* the verdict:

- `0` — every self-check passed (the tower stood, then toppled, energy was conserved/dissipated)
- `1` — a self-check failed
- `2` — precision mismatch: the linked library is not double precision
- `3` — NaN or a `dWorldQuickStep` allocation failure mid-run (the world did not advance)

In CI you only need the exit code; the per-step diagnostics and the `=== self-check ===`
table are for a human reading the log when something fails.

## Walkthrough

Every excerpt below is verbatim from the shipped asset and cited as `assets/jenga.cpp:NN`.

### Precision guards (compile-time + runtime)

Two guards bracket the whole ABI question. The `static_assert` fails the *build* if this
translation unit was compiled with the wrong precision macro (a `float` `dReal` is a broken
ABI against a double-precision library):

```cpp
static_assert(sizeof(dReal) == sizeof(double),
              "dReal must be double — this TU was compiled with the wrong precision macro");
```
— `assets/jenga.cpp:43`

The matching runtime guard catches a wrong *library* at startup and exits `2` rather than
crashing later. `dCheckConfiguration` is the supported way to query the linked build
(`include/ode/common.h:573`; the `"ODE_double_precision"` token is documented at
`include/ode/common.h:542`):

```cpp
    if (!dCheckConfiguration("ODE_double_precision")) {
        std::fprintf(stderr, "FATAL: linked ODE library is not double precision.\n");
        dCloseODE();
        return 2;
    }
```
— `assets/jenga.cpp:307`

### World setup — the tuning that makes a 54-box stack stable

All world tokens are set **before** any body is created, because auto-disable and damping
defaults are copied into each body at creation time (`assets/jenga.cpp:317`):

```cpp
    dWorldSetGravity(world, 0, 0, -GRAVITY);
    dWorldSetERP(world, 0.2);
    dWorldSetCFM(world, 1e-5);
    dWorldSetQuickStepNumIterations(world, QS_ITERS);
    dWorldSetContactSurfaceLayer(world, 0.001);     // anti-jitter at rest
    dWorldSetContactMaxCorrectingVel(world, 1.0);   // anti-popping
    dWorldSetLinearDamping(world, 0.01);            // bleed residual motion so it settles
    dWorldSetAngularDamping(world, 0.05);
    dWorldSetMaxAngularSpeed(world, 200);
```
— `assets/jenga.cpp:319`

`dWorldSetQuickStepNumIterations` is the load-bearing one: QuickStep is an iterative SOR
solver (`include/ode/objects.h:462`), and the default 20 iterations under-converge a deep
contact island, so the stack would sag or sink. Auto-disable lets settled blocks stop being
simulated (`include/ode/objects.h:720,748`):

```cpp
    dWorldSetAutoDisableFlag(world, 1);
    dWorldSetAutoDisableLinearThreshold(world, 0.02);
    dWorldSetAutoDisableAngularThreshold(world, 0.02);
    dWorldSetAutoDisableAverageSamplesCount(world, 10);
    dWorldSetAutoDisableSteps(world, 10);

    dCreatePlane(space, 0, 0, 1, 0);        // static ground (a geom with no body)
```
— `assets/jenga.cpp:329`

The ground is a `dCreatePlane` geom with **no body** — the canonical static-collider idiom.

### Tower construction — bodies, box mass, box geoms

`buildTower` lays 18 layers; odd layers run along x, even along y (rotated 90 degrees),
exactly like real Jenga. Each block is the standard ODE quartet — create body, set position,
set a box mass (`dMassSetBox`, `include/ode/mass.h:65`), create a box geom and bind it to the
body with `dGeomSetBody`:

```cpp
            dBodyID body = dBodyCreate(world);
            dBodySetPosition(body, x, y, z);

            dMass m;
            dMassSetBox(&m, BLK_DENS, lx, ly, lz);
            dBodySetMass(body, &m);

            dGeomID g = dCreateBox(space, lx, ly, lz);
            dGeomSetBody(g, body);
```
— `assets/jenga.cpp:239`

A tiny vertical `GAP` (0.0008 m) seeds the blocks just barely apart so they settle *onto*
each other rather than starting interpenetrating (`assets/jenga.cpp:62,227`).

### Near-callback — contacts with soft, rolling friction

`dSpaceCollide` hands a broadphase pair to `nearCallback`, which runs the narrow phase
(`dCollide`) and turns each contact point into a contact joint. Two correctness details:
the `dAreConnectedExcluding` guard (`include/ode/objects.h:3499`) is future-proof and,
critically, *excludes contact joints* so stacked bodies keep generating fresh contacts every
step; and surface params are filled only for the `n` real contacts, not all 16 slots:

```cpp
    dContact contact[MAX_CONTACTS];
    int n = dCollide(o1, o2, MAX_CONTACTS, &contact[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) {
        contact[i].surface.mode = dContactApprox1 | dContactSoftCFM |
                                  dContactSoftERP | dContactRolling;
        contact[i].surface.mu       = MU;
        contact[i].surface.soft_cfm = SOFT_CFM;
        contact[i].surface.soft_erp = SOFT_ERP;
        contact[i].surface.rho      = RHO;     // rolling friction
        contact[i].surface.rho2     = RHO;
        contact[i].surface.rhoN     = RHO;     // spinning friction
```
— `assets/jenga.cpp:115`

`dContactRolling` and the `rho/rho2/rhoN` fields are declared in `include/ode/contact.h:45,62-64`;
`dContactApprox1` (the friction-pyramid approximation) at `include/ode/contact.h:51`.

### The per-step loop

One step = collide, integrate, discard contacts. The return of `dWorldQuickStep` is
checked: `0` means the stepper could not allocate and the world *did not advance*, so the
caller must stop (`include/ode/objects.h:427`):

```cpp
    g_contactsThisStep = 0;
    dSpaceCollide(space, 0, &nearCallback);
    if (!dWorldQuickStep(world, STEP)) {
        g_stepFailed = true;
        std::fprintf(stderr, "FATAL: dWorldQuickStep allocation failed — "
                             "world did not advance\n");
    }
    dJointGroupEmpty(contactgroup);
```
— `assets/jenga.cpp:278`

The contact group is emptied every step — contact joints are single-use, regenerated each
frame from fresh collision results.

### Energy from first principles (body-frame inertia)

ODE provides no energy query, so the harness computes it. The subtle part is rotational KE:
`dMass.I` is stored in *body* coordinates, so angular velocity (returned in the world frame)
must be rotated into the body frame first — `omega_body = R^T * omega_world` — using the
body->world matrix from `dBodyGetRotation` (a 4x3 row-major matrix, `include/ode/objects.h:1106`):

```cpp
    const dReal *w = dBodyGetAngularVel(b);  // world frame
    const dReal *R = dBodyGetRotation(b);    // body->world, row-major 3x4 (R[r*4+c])

    // omega in body frame: wb[i] = sum_r R(r,i) * w(r)  == (R^T w)
    double wb[3];
    for (int i = 0; i < 3; i++)
        wb[i] = R[0*4+i]*w[0] + R[1*4+i]*w[1] + R[2*4+i]*w[2];
```
— `assets/jenga.cpp:150`

Potential energy uses `z = 0` (the ground plane) as reference, so total mechanical energy
`E = KE + PE` already includes gravity (`assets/jenga.cpp:166`). That choice is what makes
the later "E must be non-increasing after launch" check meaningful.

### Phase 1 — prove the tower STOOD

The settle loop requires a *sustained* quiet streak (`QUIET_STREAK` = 30 consecutive quiet
steps), not a single lull — a settling stack passes through transient velocity-reversal
dead-spots that look quiet for one frame:

```cpp
        quiet = atRest(s) ? quiet + 1 : 0;
        if (i > 40 && quiet >= QUIET_STREAK) break;
```
— `assets/jenga.cpp:371`

When it settles, the harness records the standing top height **and each block's settled
pose** — this snapshot is the baseline the topple check measures against:

```cpp
    std::vector<double> sx(blocks.size()), sy(blocks.size()), sz(blocks.size());
    for (size_t k = 0; k < blocks.size(); k++) {
        const dReal *p = dBodyGetPosition(blocks[k]);
        sx[k] = p[0]; sy[k] = p[1]; sz[k] = p[2];
    }
```
— `assets/jenga.cpp:380`

### Phase 2 + 3 — throw the ball, then track energy through impact

`throwBall` creates the sphere with `dMassSetSphereTotal` (`include/ode/mass.h:53`), aims it
at a fraction of tower height for leverage, and gives it an initial linear velocity straight
at the tower (`assets/jenga.cpp:255-270`). After launch *no further external work is added*,
so total mechanical energy must be non-increasing. The run loop captures the **fixed launch
baseline** and the **worst single-step rise** in E (spurious solver energy):

```cpp
    const double launchE = snapshot(blocks, ball).e();
    double prevE = launchE, peakE = launchE, worstRise = 0.0;
```
— `assets/jenga.cpp:406`

```cpp
        double E = s.e();
        if (E - prevE > worstRise) worstRise = E - prevE;
        prevE = E;
        if (E > peakE) peakE = E;
```
— `assets/jenga.cpp:422`

### The self-check harness — checks that can actually fail

After the run, the harness emits eight checks into a `Check` table. The verdict is the
AND of all of them and becomes the exit code. The two phase checks are the heart:

```cpp
    bool stood = settleRested &&
                 standingTopZ >= 0.95 * initialTopZ &&
                 standingTopZ <= 1.05 * initialTopZ;
```
— `assets/jenga.cpp:470`

`stood` fails if the tower *sank* (under-converged solver), *floated* (energy injection),
or never came to rest — it is a tight +/-5% band around the as-built height AND a rest
requirement, so it is not a freebie.

```cpp
    add("tower toppled after impact", finalTopZ < 0.6 * standingTopZ, d);
    ...
    add("majority of blocks displaced", toppleFrac > 0.5, d);
```
— `assets/jenga.cpp:480, 484`

The topple check fails if the ball *missed* or *bounced off* without knocking the tower
over. `toppleFrac` is measured against the per-block settled-pose baseline recorded in
phase 1: a block counts as displaced only if it moved more than `DISPLACE_THRESH` (0.30 m)
from where it settled (`assets/jenga.cpp:444-450`). The two energy checks close the loop:

```cpp
    add("no spurious energy injection", worstRise <= ENERGY_RISE_TOL, d);
    ...
    add("net energy dissipated", fin.e() < launchE, d);
```
— `assets/jenga.cpp:488, 492`

The verdict and exit code:

```cpp
    bool allOk = true;
    for (const Check &c : checks) {
        std::printf("  [%s] %-32s  %s\n", c.ok ? "PASS" : "FAIL",
                    c.name.c_str(), c.detail.c_str());
        allOk = allOk && c.ok;
    }
    ...
    return allOk ? 0 : 1;
```
— `assets/jenga.cpp:495, 516`

## What to copy into your own program

The reusable skeleton — generalize the constants, keep the structure:

1. **Guard the ABI**: `static_assert(sizeof(dReal) == sizeof(double), ...)` at file scope +
   a runtime `dCheckConfiguration("ODE_double_precision")` returning a distinct exit code.
2. **Set every world token before creating bodies** (gravity, ERP/CFM, QuickStep iterations,
   contact surface layer, max correcting vel, damping, and the full auto-disable group).
   Raising QuickStep iterations is the single biggest lever for stacking stability.
3. **File-scope `world` / `space` / `contactgroup` + a `g_*` accumulator** because
   `dSpaceCollide`'s callback is a C function pointer and cannot capture. Sum contacts and
   raise an error flag (`g_stepFailed`) from inside the callback / step helper.
4. **A near-callback** that runs `dCollide` into a `dContact[N]`, sets `.surface` only for
   the `n` real contacts, guards with `dAreConnectedExcluding(..., dJointTypeContact)`, and
   creates one contact joint per point; empty the joint group each step.
5. **A one-line step helper** that checks `dWorldQuickStep`'s return and stops on `0`.
6. **A whole-system `Snapshot`** (KE split lin/rot, PE, peak speed, awake count, a NaN flag)
   computed by iterating bodies — your single source of truth for both the live diagnostics
   and the final checks.
7. **Two-phase verification with a recorded baseline**: prove the desired *before* state
   (it stood / it was placed), snapshot per-body poses, perturb, then prove the desired
   *after* state (it toppled / it moved) *relative to that snapshot*, plus an
   energy-conservation check anchored to a fixed launch baseline. AND the checks into one
   bool and return it as the exit code.

## Gotchas this program handles

- **A passing check that cannot fail is worthless.** Every check is designed to be tripped
  by a *plausible* bug: `stood` (`assets/jenga.cpp:470`) trips if the solver under-converges
  and the stack sinks; `toppled` (`assets/jenga.cpp:480`) trips if the ball misses;
  `no spurious energy injection` (`assets/jenga.cpp:488`) trips if QuickStep adds energy. A
  check like "topZ > 0" would always pass and prove nothing — these don't.
- **Energy must be measured against a FIXED baseline, not the previous frame only.** The
  net-dissipation check compares the final state to `launchE` captured once at launch
  (`assets/jenga.cpp:406, 492`); the injection check tracks the worst *single-step* rise
  (`assets/jenga.cpp:422`). A drift check against only the prior frame could be fooled by a
  slow ramp; the fixed anchor cannot. ODE rule: after the launch impulse no external work is
  done, so `E = KE + PE` must be non-increasing — this only holds because PE references
  `z = 0` and gravity is folded into PE (`assets/jenga.cpp:166`).
- **Rotational KE needs the body frame.** `dMass.I` is in body coordinates
  (`include/ode/mass.h`), but `dBodyGetAngularVel` returns world-frame omega; rotating it by
  `R^T` first (`assets/jenga.cpp:150-156`) is required or the energy numbers are garbage and
  the energy checks become meaningless.
- **"Settled" must be a sustained streak, not one quiet frame.** A toppling stack has
  transient near-zero-velocity instants; requiring `QUIET_STREAK` consecutive quiet steps
  (`assets/jenga.cpp:371, 430`) avoids declaring rest mid-collapse.
- **Raise QuickStep iterations for deep contact islands.** QuickStep's default 20 SOR
  iterations under-converge a 54-box stack; 80 (`assets/jenga.cpp:322`,
  `dWorldSetQuickStepNumIterations` at `include/ode/objects.h:462`) lets the resting-contact
  forces balance so the tower neither sinks nor jitters.
- **Rolling AND spinning friction so the system can actually come to rest.** Without
  `dContactRolling` + `rho/rho2/rhoN` (`assets/jenga.cpp:121-128`,
  `include/ode/contact.h:45,62-64`), the heavy ball rolls/spins forever and the run never
  satisfies `atRest`, so the "system settled" check would never pass and the program would
  hang against its time cap.
- **Check the stepper's return value.** `dWorldQuickStep` returns `0` on an allocation
  failure meaning the world did NOT advance (`include/ode/objects.h:427`); the helper sets
  `g_stepFailed` and the loop breaks (`assets/jenga.cpp:280, 358, 414`) instead of silently
  integrating a frozen world.
- **`dAreConnectedExcluding(..., dJointTypeContact)`** — excluding the contact type is what
  lets stacked bodies regenerate contacts every step; a plain `dAreConnected` would suppress
  them and the stack would fall through itself (`assets/jenga.cpp:109`,
  `include/ode/objects.h:3499`).

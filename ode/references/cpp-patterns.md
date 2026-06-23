# Using ODE well from C++ (the dominant path)

> "C++ ODE" = the C API written in a `.cpp`, built with the `c++` driver (`references/building-and-running.md`).
> This file is how to use that C API *idiomatically and safely* from C++. It is NOT the optional `odecpp.h`
> wrapper classes — those live in `references/cpp-api.md` and the field never needed them. Every pattern here
> is lifted from real, tested programs (`assets/arm6dof.cpp`, `assets/jenga.cpp`, `assets/buggy_ramp.cpp`).

## 1. The #1 C++ compile trap: the near-callback must be a function pointer

`dSpaceCollide` takes a `dNearCallback*` — a **C function pointer**:
`typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2);` (`include/ode/collision_space.h:49`).

So your callback must be a **free function**, a **`static` member**, or a **non-capturing lambda** (which
decays to a function pointer). A **capturing lambda will not convert** and is the single most common
C++-specific compile failure. Pass per-call state through the `void *data` argument instead.

```cpp
struct World { dWorldID w; dJointGroupID cg; /* ... */ };

static void nearCallback(void *data, dGeomID o1, dGeomID o2) {   // free/static fn — required
    auto *W = static_cast<World*>(data);                         // recover state via void* data
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    dContact c[8];
    for (auto &x : c) { x.surface.mode = dContactApprox1; x.surface.mu = 1.0; }
    int n = dCollide(o1, o2, 8, &c[0].geom, sizeof(dContact));
    for (int i = 0; i < n; i++) dJointAttach(dJointCreateContact(W->w, W->cg, &c[i]), b1, b2);
}
// ...
dSpaceCollide(space, &myWorld, &nearCallback);   // pass state as the data pointer, NOT via capture
```

## 2. RAII over raw handles (hand-rolled, move-only)

ODE handles (`dBodyID`, `dGeomID`, `dJointID`, …) are owning pointers. A thin RAII wrapper makes teardown
exception-safe — but it **must be move-only**, or a copied wrapper double-frees the same handle:

```cpp
struct Body {                                   // move-only owner of a dBodyID
    dBodyID id = nullptr;
    explicit Body(dWorldID w) : id(dBodyCreate(w)) {}
    ~Body() { if (id) dBodyDestroy(id); }
    Body(const Body&) = delete;                 // no copy (would double-free)
    Body& operator=(const Body&) = delete;
    Body(Body&& o) noexcept : id(o.id) { o.id = nullptr; }
    operator dBodyID() const { return id; }     // use anywhere a dBodyID is expected
};
```

**Teardown ownership caution:** `dWorldDestroy` frees every body in the world, and `dSpaceDestroy` frees
every geom in the space. If you *also* hold RAII wrappers, do not let them double-destroy what the
world/space already freed — either destroy individual objects and skip the bulk destroy, or hold raw
handles in your containers and rely on `dWorldDestroy`/`dSpaceDestroy` for cleanup. Pick one owner.

**The teardown-ORDER foot-gun (the #1 crash when wrapping ODE in C++).** Destroying ANY ODE handle *after*
`dCloseODE()` (or after the owning `dWorldDestroy`) is **undefined behavior** — a segfault, not a clean
error. With RAII this bites silently: if a wrapped `Body`/`Primitive` *outlives* whatever calls `dCloseODE`,
its destructor runs `dBodyDestroy`/`dGeomDestroy` on torn-down state. Two safe shapes:
- **Guarantee order** (self-contained program): make the `dCloseODE`/world owner *outlive* every wrapper —
  declare bodies/geoms after the world so they destruct first (reverse declaration order), or keep them in a
  container the world owner clears before `dCloseODE`.
- **Guard for library code** (you can't control user object lifetime): keep a global "ODE is initialised"
  flag/refcount (set at `dInitODE2`, cleared at `dCloseODE`) and have every destructor **no-op when it's
  clear** — `if (g_ode_alive && id) dGeomDestroy(id);`. Field-proven: a real lpzrobots-style engine
  extraction added exactly this guard so a `Primitive` outliving its simulation stopped segfaulting.

## 3. Collections: `std::vector<dBodyID>`

Hold many objects in standard containers and iterate to apply per-step control/forces:

```cpp
std::vector<dBodyID> links;                     // or std::vector<Body> if RAII-owned
for (int i = 0; i < N; i++) { dBodyID b = dBodyCreate(world); /* mass+geom+pose */ links.push_back(b); }
// each step, before dWorldStep:
for (dBodyID b : links) dBodyAddForce(b, 0, 0, lift);
```

## 4. Per-step control loop (read → compute → apply → step)

The order is load-bearing: read state, compute the command, **apply it BEFORE `dWorldStep`**, then step:

```cpp
for (int i = 0; i < STEPS; i++) {
    // 1. read: dBodyGetPosition / dBodyGetLinearVel / dJointGetHingeAngle ...
    // 2. compute the command (controller)
    // 3. apply: dJointSetHingeParam(j, dParamVel, target); dJointSetHingeParam(j, dParamFMax, torque);
    dSpaceCollide(space, &W, &nearCallback);
    if (!dWorldStep(world, DT)) break;          // fixed DT; check the return
    dJointGroupEmpty(W.cg);
}
```

A **velocity servo** in ODE is exactly the pair `(dParamVel = target rate, dParamFMax = max torque)`;
`dParamFMax = 0` makes a motor inert (`include/ode/common.h:443,446`).

### Discrete-stability trap: `dParamVel` SETS velocity — don't put a `-KD*pdot` term in it
The header documents `dParamVel` as the **desired motor velocity** the motor reaches within `dParamFMax`
force, not a force you add (`include/ode/objects.h:1642-1647`; symbols at `include/ode/common.h:443,446`).
So commanding `dParamVel` *replaces* the joint velocity for the next step — it is a velocity assignment, not
a torque. A naive PD position servo `v = KP*(p_des - p) - KD*pdot` therefore turns the damping term into a
**discrete self-feedback** on the velocity it just set: with `pdot ≈ v_n`, the next command is
`v_{n+1} ≈ -KD*v_n`, a sign-flipping oscillation whose stability boundary is **near `KD ≈ 1` but is
DT-dependent** (field-measured ≈ 0.98) and **bounded by `dParamFMax`/`VMAX`** — so you get growing
**chatter**, not unconditional blow-up.
This is observed/empirical — the header states the velocity-assignment semantics but is silent on the loop
stability. **Keep `KD < 1`** (and use `dParamFMax`, not `KD`, to bound how hard the servo pushes). Field
data: a Stewart-platform velocity servo with `KD = 1.5` chattered at ~1 m/s; dropping to `KD = 0.2`
improved every metric 100–1000×. (To get true PD *damping* instead, drive the joint with force —
`dBodyAddForce`/`dJointAddHingeTorque` — where `-KD*pdot` is a real viscous term.)

### Reusable velocity servo (production refinements)
Packaging a motorized joint as a position→velocity servo, the details that make it stable (lpzrobots
`motors/oneaxisservo.cpp`):
- **One-step anti-overshoot clamp.** After computing the command velocity `v` toward `p_des`, cap it so it
  can't pass the target in one step: `if (dt*|v| > |p_des - p|) v = (p_des - p)/dt;`. Kills the residual
  jitter a pure P-servo leaves at the setpoint (`oneaxisservo.cpp:142-159`).
- **The speed cap IS the gain.** `dParamVel` is clamped to `maxVel` with P-gain `≈ maxVel/2` — one knob sets
  both proportional stiffness and top speed (`oneaxisservo.cpp:89`).
- **Set hard joint stops ~30% OUTSIDE the servo's commanded travel** (`dParamLoStop`/`dParamHiStop`) so the
  servo never fights its own limit; a stop *inside* the commanded range makes motor and constraint oscillate.

### The actuated-joint pattern (kinematic constraint + parallel user-mode AMotor)
To torque-control a hinge DOF while keeping the structural constraint clean: build the `dHinge` with its
own motor OFF (`dParamFMax = 0`), and attach a separate `dAMotor` in `dAMotorUser` mode (NumAxes=1) to the
**same two bodies**; each step, re-set the AMotor axis from the live hinge axis (`rel = 0`, global frame)
then command `dParamVel`/`dParamFMax`. Worked example: `references/examples/arm6dof.md` (`assets/arm6dof.cpp:333-340,258-263`).

### The sign trap (verify, don't assume)
A velocity-controlled `dAMotor`'s `dParamVel` sign depends on `dJointAttach` body order: a **world-attached**
motor (`dJointAttach(am, 0, b)`) and an **interior two-body** motor (`dJointAttach(am, b0, b1)`) have
**opposite** effective sense. Calibrate it with a tiny gravity-off probe (`assets/probe_sign.cpp` → +1,
`assets/probe_sign2.cpp` → −1) and store a per-joint sign. See `references/components/joint-amotor.md`.

## 5. The canonical C++ program shape: build headless + self-checking

Make every program a `#ifdef HEADLESS` two-build: the same physics renders (optional) or runs headless and
**grades itself**. `main()` returns `ok ? 0 : 1` (and `2` on precision mismatch) so it is CI/agent-gradable
with no window. Skeleton in `references/rendering-and-headless.md` §1; design discipline in
`references/foundations/verifying-simulations.md`; complete examples `assets/headless_example.c`,
`assets/arm6dof.cpp`, `assets/buggy_ramp.cpp`.

## 6. Linkage
`#include <ode/ode.h>` already guards the C API with `extern "C"` for C++ (`include/ode/README`). A user
callback passed to ODE just needs to be a compatible function pointer (§1). Mixing `.c` and `.cpp`
translation units links fine because libode is itself C++ (`references/building-and-running.md` §2).

## Things to never invent
- A capturing lambda as a `dNearCallback` (won't compile) — use a free/static fn + `void *data`.
- A copyable RAII handle wrapper (double-frees) — make it move-only.
- A motor that drives with only `dParamVel` set — it needs `dParamFMax > 0`.

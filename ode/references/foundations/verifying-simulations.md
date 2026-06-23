# Verifying an ODE simulation (it compiled & ran is the weakest evidence)

> The harness is the **tool** (`references/rendering-and-headless.md` §1: run headless, print state, return
> an exit code). This file is the **method** — how to design the checks so a *broken* sim actually fails.
> Every principle is embodied by a shipped, field-tested program; cite them as worked probes:
> `assets/probe_sign.cpp`, `assets/buggy_ramp.cpp` (`-DHEADLESS`), `assets/jenga.cpp`, `assets/headless_example.c`.

A program that prints `PASS` while being physically wrong is worse than no check — it manufactures false
confidence. The five principles below are what separate a real verdict from a rubber stamp.

> **Don't re-derive it — drop in `assets/verify_harness.hpp`.** Header-only, depends only on `<ode/ode.h>`,
> and packages every principle below as a primitive: `finite()` (NaN trip-wire), `total_energy()`,
> `pose_digest()` (determinism fingerprint), `min_z()`/`max_speed()`, `joint_separation()` (constraint
> satisfaction), `differs()` (the A/B confound guard), and a `Checks` accumulator that prints each PASS/FAIL
> and returns the exit-code verdict. Worked, runnable usage: `assets/harness_selftest.cpp`.

## 1. Fixed-baseline energy — assert E never grows (passive/conservative systems only)
Total mechanical energy `E = KE + PE` must fall (contacts dissipate) then **hold**; a *growing* E is the
universal signature of instability (bad ERP/CFM, too-large `DT`, precision mismatch). **This check applies
only to passive systems.** A thrust- or motor-driven craft (quadrotor, walking robot, conveyor) *adds*
energy by design, so energy is not bounded — for those, drop the energy check and assert **bounded state +
reached-target** instead (final pose within tolerance, speed/tilt bounded, no NaN, no tunnel). Record a baseline
`E0` after settling (or at launch) and assert `E(t) ≤ E0 + ε` for all `t` — against a **fixed** baseline,
not a running max (a running max can never fail).
- **Good:** `E0` = energy at impact launch; FAIL if any later step exceeds it by > tolerance (`assets/jenga.cpp`).
- **Bad:** run 500 steps, print "done", exit 0 — a slowly-exploding sim passes.

## 2. Two-phase check — transient vs rest
Separate a **transient** phase (motion: PE→KE, impacts) from a **rest** phase (settled) and assert different
invariants in each; one global assertion misses half the bugs.
- **Transient:** no NaN; `zmin` stays above the floor (no tunneling); `vmax` bounded.
- **Rest:** joint anchor separation tiny; velocities ≈ 0; `E` flat.
- **Good (jenga):** prove the tower *stood* (else "it fell" is meaningless), then prove the impact *toppled*
  it (top height collapses + a majority of blocks displace) while staying physical.
- **Bad:** a single end-of-run snapshot that catches the system mid-bounce and reports a "reasonable" pose.

## 3. Falsification over confirmation — try to break the claim; probe unknown conventions
Design the check to **disprove** the claim, and when a sign/axis/convention is uncertain, **measure it**
rather than assume it matches GLM/Bullet/your textbook.
- **Good:** predict "positive AMotor `dParamVel` → tip moves −Z", command it, **measure** the tip z, FAIL if
  the sign disagrees (`assets/probe_sign.cpp`). For collision: assert the body actually *stops at* the floor,
  not merely that a contact was generated.
- **Bad:** assume ODE's quaternion/axis/sign convention matches another library and ship a mirror-imaged sim
  that "looks plausible."
- **Beware the confound — a negative control can pass for the wrong reason.** A check only falsifies if it
  *isolates the variable under test*. *Field example:* an "actuators OFF → the platform collapses" negative
  control passed even with **no load** — gravity sag alone tripped the drop threshold, so it never tested
  the load at all. The load's true signature was **tilt** (72° loaded vs 0.02° gravity-only). Fix: make the
  check **differential** — change exactly ONE variable and require the **difference** (loaded vs gravity-only
  vs controlled), not an absolute threshold. Then **prove the check bites**: perturb the input and confirm
  the check flips to FAIL. *A fix that doesn't restore falsifiability is theater.*
- **One invariant can be BLIND to a whole bug class — pick checks with DISJOINT blind spots.** A bug that *preserves* the quantity you assert is invisible to that assertion. *Field example (double pendulum):* flipping the gravity sign still does not grow total energy — gravity is conservative either direction, and the energy check recomputes PE from the same `g` you configured — so an energy-non-growth check PASSES the bug — only a check whose blind spot is disjoint (here the analytic normal-mode frequency, or "does the bob settle below where it started") catches it. This differs from the independent-oracle rule below: that guards against a verifier sharing *code/context*; this guards against the *metrics themselves* sharing a blind spot. Use ≥2 checks whose failure modes do not coincide.

## 4. Determinism — same seed + fixed DT + same build ⇒ same trajectory
ODE is deterministic under a fixed step — but at the **PROCESS** level: launch the program twice and compare
final positions to a recorded baseline. **In-process reruns can legitimately differ** under
`dWorldQuickStep`+`dHashSpace` (broadphase pairs are ordered by geom *pointer address*, and SOR is
ULP-order-sensitive, so a rebuilt world's shifted heap addresses reorder contacts) — so compare two PROCESS
launches, not rerun-vs-rerun. Cross-process non-determinism means uninitialized memory, a variable `DT`
sneaking in (use a fixed `DT`, never wall-clock — `references/foundations/stepping-and-stability.md`),
a threading data race, or unseeded RNG. A flaky `PASS/FAIL` is itself a bug report.

## 5. No tautology — assert an emergent consequence, never the value you set
Reading back what you just set proves nothing — it is the input echoed. A real check lets the solver run and
tests something it had to **compute**.
- **Bad:** `dWorldSetGravity(w,0,0,-9.81); assert(dWorldGetGravity(w)[2]==-9.81);` — that "verifies" nothing.
- **Good:** assert a dropped sphere's settled height ≈ its radius (the *solver* placed it on the floor), or
  that a hinge-constrained pair's two anchor read-backs coincide **after** stepping (the constraint held).

## The checklist
A trustworthy ODE self-check has: fixed `DT`; a NaN trip-wire every step; a no-tunnel floor check; a
fixed-baseline energy assertion; phase-appropriate invariants (transient vs rest); a falsifiable target
(it can actually FAIL); determinism across two runs; and a process **exit code** as the verdict
(`0` PASS / `1` FAIL / `2` precision mismatch). If you cannot describe what would make your check FAIL, it
is not a check. Cross-reference the harness skeleton in `references/rendering-and-headless.md` §1 and the
C++ program shape in `references/cpp-patterns.md` §5.

## Reusable verification techniques (every build re-derived these)

- **Constraint-satisfaction metric.** A joint's two anchor read-backs coincide only while the constraint
  holds, so `|dJointGetBallAnchor − dJointGetBallAnchor2|` (and the hinge/slider equivalents) is the live
  joint-stretch error — assert it stays tiny at rest (µm scale; a blown joint diverges to metres). This is
  the single best *emergent* check that an **open** chain / single joint is intact — **but it SATURATES for
  over-constrained / closed loops**: ODE's ERP heals the positional error every step, so `|anchor−anchor2|`
  stays ~µm no matter how badly the loop is broken (field: break 0.05→0.50 m all read ~2-3e-5 vs 1.2e-6
  intact). For a **closed loop the violation lives in the REACTION FORCE, not the position** — set
  `dJointSetFeedback` on the closing joint and assert its `dJointFeedback` force stays bounded (field:
  25.7 N intact → 190-640 N broken). Use BOTH observables — the disjoint-blind-spot rule (§3) applied to this
  very check. (A mis-placed *single* anchor still reads coincident, since both read-backs derive from one
  stored constraint — separation catches a geometric *contradiction* among constraints, not one bad anchor.) (Symbols in `include/ode/objects.h`;
  the line is approximate — grep your installed header.)
- **Determinism fingerprint.** Don't eyeball "same trajectory" — hash it: fold every body's final position +
  orientation into one digest (e.g. FNV-1a over the `double`s) and compare bit-for-bit. **Run the two passes
  as separate PROCESS launches**, not two in-process reruns — under `dWorldQuickStep`+`dHashSpace` an
  in-process rerun reorders contacts (broadphase keyed on geom *pointer address*) and drifts at the ULP level
  *even single-threaded*, so a same-process compare can false-FAIL. A *cross-process* mismatch is a real bug
  (uninitialised memory, a variable `DT`, a threading race — a genuinely multi-threaded step is *not*
  bit-reproducible, see `references/threading.md`).
- **Auto-disable reads as zero motion.** A body ODE has auto-disabled reports ~zero velocity, so a
  "settled = low speed" check passes *trivially* the moment a body sleeps. For a verification metric, either
  keep auto-disable off, or assert the body actually reached its target pose — not just that it stopped.
- **Coverage — a check must actually exercise what it claims.** A self-check that never touches a code path
  proves nothing about it (field: a ray-sensor harness that "verifies obstacle detection" but whose rays
  never hit the sphere proves nothing about sphere casts). Count the hits per object type / branch and assert
  each was exercised, or the green is hollow.
- **An independent oracle must share no CODE *and* no CONTEXT.** A cross-check is only independent if it
  reaches the answer by a disjoint route (a from-scratch RK4 with no ODE, an analytic formula, a hand
  calc). *Field trap:* an agent forked a sub-verifier that **inherited its context and mirrored its own
  summary** instead of recomputing — "a verification gap, not a result." If you delegate verification, give
  the verifier the inputs and the claim, never your reasoning.
- **Beware the pipe that eats the verdict.** `./prog | grep …` returns *grep's* exit code, not the
  program's — so a FAIL can silently read as success in CI. Capture the program's own exit (`./prog; echo $?`
  or `${PIPESTATUS[0]}` after a pipe); the exit code *is* the verdict.

## Computing total energy — the recipe ODE doesn't give you

Principle #1 needs `E = KE + PE`, but **ODE never computes energy** — you must reconstruct it from the
reported state. The two traps every build hit: `dMass.I` is a `dMatrix3` stored **3×4 row-padded** (index
`I[row*4+col]`, the 4th column is padding), and angular velocity is world-frame so you transform it into the
body frame with `Rᵀ`. This helper is correct and reusable:

```c
// E = sum over bodies of (linear KE + rotational KE + gravitational PE), recomputed from ODE's state.
// g is the gravity vector you passed to dWorldSetGravity, e.g. {0,0,-9.81}.
static double totalEnergy(const dBodyID *bodies, int n, const dReal g[3]) {
    double E = 0.0;
    for (int k = 0; k < n; k++) {
        dBodyID b = bodies[k];
        dMass m; dBodyGetMass(b, &m);                 // m.mass = scalar; m.I = dMatrix3 (3x4, row-padded)
        const dReal *v = dBodyGetLinearVel(b);        // world frame
        const dReal *w = dBodyGetAngularVel(b);       // world frame
        const dReal *p = dBodyGetPosition(b);
        const dReal *R = dBodyGetRotation(b);         // 3x4 row-major: R[row*4 + col]
        double ke = 0.5 * m.mass * (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);   // linear KE
        double wb[3];                                  // omega in BODY frame: w_body = R^T w_world
        for (int i = 0; i < 3; i++)
            wb[i] = R[0*4+i]*w[0] + R[1*4+i]*w[1] + R[2*4+i]*w[2];
        double Iw[3];                                  // I * w_body  (m.I is 3x4 row-padded)
        for (int i = 0; i < 3; i++)
            Iw[i] = m.I[i*4+0]*wb[0] + m.I[i*4+1]*wb[1] + m.I[i*4+2]*wb[2];
        ke += 0.5 * (wb[0]*Iw[0] + wb[1]*Iw[1] + wb[2]*Iw[2]);            // rotational KE = ½ ωᵀ I ω
        double pe = -m.mass * (g[0]*p[0] + g[1]*p[1] + g[2]*p[2]);        // PE = -m (g·r); for g_z<0 this is +m|g|z
        E += ke + pe;
    }
    return E;
}
```

Cites: `dBodyGetMass`/`dBodyGetLinearVel`/`dBodyGetAngularVel`/`dBodyGetRotation`/`dBodyGetPosition`
(`include/ode/objects.h`); `dMass.I` is a `dMatrix3` (`include/ode/mass.h`); `dMatrix3` is 3×4 row-padded
(`include/ode/common.h`). Record `E0` once after settling/at-launch and assert `E(t) ≤ E0 + ε` thereafter.
An independent cross-check (a separate RK4 integrator with **no ODE**, or an analytic normal-mode frequency)
turns "energy is bounded" into "the dynamics are right" — see the double-pendulum example.

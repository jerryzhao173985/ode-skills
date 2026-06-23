# Foundations: known issues & real-world gotchas (from the ODE issue tracker)

> Grounded in the upstream GitHub tracker (`github.com/odedevs/ode/issues`). These are *engine limitations and recurring user pain* from real bug reports — distinct from the day-to-day API-misuse mistakes called out across the `references/` files. Each item names the issue and the practical lesson. **These reflect upstream bug reports — verify against the current source before treating any as permanent.**

## Setup / lifecycle

- **Step crashes if ODE isn't initialized.** Calling `dWorldStep`/`dWorldQuickStep` without `dInitODE`/`dInitODE2` first segfaults on the first step — the single most common beginner mistake. ([#49]) → Always `dInitODE2(0)` before anything; see `references/world-and-stepping.md`.

## Stability / solver tuning

- **Don't assume a step "just works."** Non-trivial sims can become unstable after solver internals change (e.g. constraint-reordering batches); stability is sensitive to step size, ERP/CFM, and iteration count. Tune, don't assume. ([#51]; see `references/foundations/stepping-and-stability.md`)
- **Convex/convex resting is fragile.** Resting one convex shape on another often yields only ONE contact point (libccd returns 0–1), so the body wobbles/jumps corner-to-corner. A single contact can't stably resolve a resting contact. ([#56])
- **Contacts can't perfectly stick.** The contact normal row uses LO=0, HI=∞, so it only pushes *outward* — bodies always restore a little energy even at `bounce=0`; perfectly inelastic "stick" is not achievable with a contact joint alone. ([#88])
- **"ODE Message 3: LCP internal error, s <= 0" is a *non-fatal* warning, not a crash.** It is `dWorldStep`'s Dantzig solver hitting a non-positive pivot — `dMessage(d_ERR_LCP, …)` at `ode/src/lcp.cpp:1091` (`d_ERR_LCP=3`, `include/ode/common.h:378`, hence the "Message 3" prefix); the solver zero-fills the remaining unknowns and bails to the partial solution (`lcp.cpp:1087-1098`), and because `dMessage` returns normally (it is **not** `ODE_NORETURN` like `dError`, `include/ode/error.h:56`) **it does not abort the process.** Per the FAQ it usually means an object was rammed too hard; the fixes are to reduce the object's mass or change the timestep, and the occasional message can be ignored. A *flood* every step means the solve is degenerate — fix the underlying instability (scaling / ERP / CFM / stepper choice; see `references/foundations/stepping-and-stability.md`). (FAQ <https://www.ode.org/wiki/index.php?title=FAQ>; disposition source-verified.)

## Collision robustness / dropped contacts

- **Small/fast objects tunnel.** Spheres small relative to a heightfield/trimesh cell tunnel through convex edges/ridges (contacts culled by bounds checks). Workaround: make the cell size smaller than the smallest dynamic object. ([#71]; see `references/trimesh-heightfield.md`)
- **Request enough contacts.** When a collider (e.g. GIMPACT) produces more contacts than your `flags` cap, ODE may trim the surplus **without sorting by depth/importance**, keeping effectively random contacts → penetration. Ask for enough, and be aware of the cap. ([#36])
- **Trimesh↔trimesh skips the per-triangle callback** in some versions (callback wiring missing), so per-triangle filtering doesn't work for tri-tri pairs. ([#23])
- **`dMassSetTrimesh` uses the world transform**, not the geom's local/offset frame — building mass from a trimesh placed away from the origin yields wrong inertia. ([#5])
- **Capsule-vs-box returns a garbage normal on deep penetration.** When the capsule axis penetrates the box deeper than the capsule radius, the closest points coincide and `dCollideCapsuleBox` yields an undefined (non-unit / wrong-direction) `contact.geom.normal` → the body is kicked the wrong way. Avoid deep capsule-box interpenetration (smaller step / softer contact / cap relative velocity). (Production-patched: lpzrobots `ode_robots/ode_patches/ode-0.11_cap_box_colbug.patch`.)

## Floating-point exceptions hidden by default

- **libccd narrowphase can divide by zero / return NaN.** `demo_convex` (`ccdVec3PointTriDist2`), cyl-vs-capsule MPR, and ray-vs-capsule can fault on near-degenerate configs — masked by the default FPU, fatal once FP exceptions are enabled. ([#52],[#72],[#26])
- **Debugging tip:** enable FP exceptions (`feenableexcept(FE_DIVBYZERO|FE_INVALID)`) and run under UBSan/Valgrind to surface NaN/Inf early. Expect a *cosmetic* "conditional jump on uninitialised value" inside the LCP solver (maintainers consider it benign but patched it). ([#85])

## Joints / motors

- **Hinge limits break near ±π.** Min/max limits are violated when the joint angle crosses ±π because the angle is clamped to [−π, π] without tracking rotation direction. Be careful setting hinge limits near ±π. ([#84])
- **AMotor `dAMotorEuler` ignores swapped bodies.** In Euler mode the global-axis computation doesn't account for `dJOINT_REVERSE`, so you must manually swap the `rel` index passed to `dJointSetAMotorAxis` (unlike `dAMotorUser`, which handles it). ([#37])
- **FAQ recipe — joint stops won't move?** Set hi, then lo, then hi again; an inconsistent stop where `hi < lo` is silently ignored. (FAQ)
- **`dJointAddSliderForce` injects an unintended torque when the body COMs aren't on the slider axis.** It is a bare wrapper that applies equal-and-opposite `dBodyAddForce` to the two bodies (`objects.h:2020-2027`); if their centres of mass are offset from the axis, the force couple has a moment arm and adds spurious torque. Apply force at the joint anchor, or drive the joint with a motor (`dParamVel`/`dParamFMax`) instead. (Production-patched: lpzrobots `ode_robots/ode_patches/ode-0.5_slider_torques.patch`.)

## Threading

- **MT requires thread-safe user callbacks.** Multithreaded stepping can invoke callbacks (e.g. `dxBody::moved_callback`) from multiple threads; non-thread-safe user code corrupts the heap and crashes. ([#30]; see `references/threading.md`)

## Build / platform

- **MSVC build break (C3861).** `_addcarry_u64`/`_subborrow_u64`/`_udiv128` not found because `ou/include/ou/atomic.h` includes `<intrin.h>` inside `namespace odeou`. Representative Windows/MSVC/vcpkg breakage. ([#78])
- **Apple Silicon link failure.** `timer.cpp` uses the deprecated Carbon `Microseconds()` symbol, unavailable on arm64. Representative macOS/cross-Apple build issue. ([#77])

## Recurring themes

1. **Instability / solver tuning** — wobble, jitter, sink, explode; worst with stacked convex (1 contact), changed reordering, trimmed trimesh contacts, and bad ERP/CFM/step choices.
2. **Collision robustness** — small objects tunnel; convex/convex and GIMPACT emit too few or unsorted contacts → penetration/wobble.
3. **FP exceptions masked by default** — div-by-zero / bad normalizations in libccd surface only with FE exceptions or UBSan; the maintainers recommend enabling them when debugging.

## See also

`references/foundations/stepping-and-stability.md` (stability tuning), `references/trimesh-heightfield.md` (trimesh/heightfield/convex), `references/threading.md` (MT safety), `references/foundations/mental-model.md` (the mental model).

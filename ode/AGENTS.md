# AGENTS.md — working in / with ODE

Operating guidance for agents writing Open Dynamics Engine (ODE) code or extending this skill. Read
`SKILL.md` first for the mental model, the canonical simulation loop, and the routing table.

## Source of truth
- The public headers `include/ode/*.h` are canonical. Cite every API claim by `file:line`. The wiki
  (ode.org) and web tutorials are secondary and **lose to the headers on conflict** — they frequently
  reference removed APIs (see `references/version-and-changelog.md`).
- Before using any `d*` symbol you are unsure of, grep it — against **your installed ODE** (most accurate):
  `ODE_INC=$(ode-config --cflags | sed 's/-I//'); grep -rn '<symbol>' "$ODE_INC/ode/"`. If it is not there,
  it does not exist — find the modern name, do not invent one. Treat this skill's repo `file:line` cites as
  approximate (0.16 repo vs your installed 0.16.x); re-grep the installed header when an exact line matters.
  Sweep every cite at once with **`python3 scripts/check-citations.py`** — it reports which cited symbols are
  absent in *your* installed ODE (e.g. the QuickStep dynamic-iteration API exists in 0.16 dev but not 0.16.6,
  and drawstuff is absent on Homebrew).
- This skill teaches the **public** surface only. `ode/src`, GIMPACT/OPCODE/libccd, and `bindings/`
  are surveyed (`references/internals-map.md`, `references/ecosystem.md`), not documented as API.

## The non-negotiables (full list + cites in SKILL.md "Hard rules")
1. A body has mass/no-shape; a geom has shape/no-mass — bridge with `dGeomSetBody`.
2. `dInitODE2(0)` first; `dCloseODE()` last; allocate per-thread data for collision.
3. Per step, in order: `dSpaceCollide` → `dWorldStep`/`dWorldQuickStep` → `dJointGroupEmpty`.
4. `dCollide` writes into a `dContact[N]` array sized to its N argument (single-`dContact` = crash).
5. Fixed timestep; check the stepper's `int` return.
6. **Define no precision macro** — `<ode/ode.h>` already matches the installed lib; assert
   `dCheckConfiguration("ODE_double_precision")` at runtime.
7. Radians (API) vs degrees (drawstuff camera `hpr`); quaternions are `[w,x,y,z]`; box lengths are full.
8. **Build with the `c++` driver + `ode-config`** — libode is a C++ library (`-x c` for `.c` sources).
   See `references/building-and-running.md`.
9. **Default to headless self-checking** — drawstuff is optional and not shipped by Homebrew.
   See `references/rendering-and-headless.md`.
10. **Integrate, don't clobber.** `ls`/Read the target dir first; if a Makefile/README/sibling demo exists,
    read it and MATCH its build convention before adding files — never overwrite project files you didn't create.

## Verifying generated ODE code
- It must compile+link with `c++ $(ode-config --cflags --libs)` (no hand-defined precision macro) — and
  `#include <ode/ode.h>` only, never an `ode/src` header.
- **Run it headless and self-check**: print energy / trajectory / joint-separation and assert PASS/FAIL
  (`references/rendering-and-headless.md`); design the checks so a *broken* sim actually FAILS — energy,
  two-phase, falsification, determinism, no tautology (`references/foundations/verifying-simulations.md`).
  "It compiled" is not "it works."
- Writing it in C++? See `references/cpp-patterns.md` — the nearCallback must be a function pointer (a
  capturing lambda won't compile), RAII handles are move-only, and controllers/sign conventions are there.
- Run the SKILL.md "Final checks" lifecycle/ordering list.
- Prefer lifting a pattern from a real demo (`references/demos-index.md` maps all 39) over writing from
  memory; the `references/examples/*.md` walkthroughs are verbatim, cited starting points.
- **Get a separate-context review — don't self-approve.** After self-checking, have a read-only reviewer
  recompute from the claim + the headers (never your reasoning); it catches harness bugs you can't see — a
  tautological check, a falsification that passes on a confound (`references/foundations/research-and-diagnosis.md` §4).

## Extending this skill
- Keep the `references/` layout: `foundations/mental-model.md` (read-first) · core API · other `foundations/`
  · `components/` (one file per joint/geom element) · deep topics · `examples/` · `pitfalls.md`.
- **A component card's value is the SEMANTICS, not the signature.** A 1-line header grep gives the exact
  (and version-accurate) signature faster than any card — so a card that is just a signature table loses to
  grep. What grep CANNOT give, and what the card must LEAD with: anchor/axis *ordering*, *units*, read-back
  *meaning*, sign *conventions*, lifetime/timing *gotchas* (e.g. a getter that returns 0 until after the
  first step), and the read-back used to *verify* the joint held. Write cards accordingly.
- Every new rule carries a `file:line` cite or a literal `[VERIFY]` marker — cites are **0.16-repo
  approximate**; re-grep the installed header for an exact line. No uncited assertions.
- Add a routing-table row in `SKILL.md` for every new reference file; never leave a phantom row.

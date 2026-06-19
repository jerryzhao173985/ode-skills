# Research, measurement & diagnosis (analysis, not just building)

> The strongest field builds didn't just compile a sim — they did research-grade work: they proved the
> bottleneck before optimizing, treated hypotheses as falsifiable, swept parameters with a negative control,
> ablated to isolate a cause, and ran a **separate-context review** that caught real bugs the author missed.
> This file makes that procedure teachable. It is the analysis companion to
> `references/foundations/verifying-simulations.md` (which covers *checking* a result); this covers
> *measuring*, *diagnosing*, and *getting independently reviewed*.

## 1. Measure, don't guess — prove the hot path before you touch it

- **Instrument before optimizing.** Time collision vs the step separately and report each as a *percent* of
  step time. (Field: an agent proved `dWorldStep` was 97–98% of step time with thousands of contacts —
  confirming the hot path with evidence, not assertion, *before* parallelizing.)
- **A mechanism claim needs a header cite AND an independent read — correct numbers don't make a wrong cause
  right.** (Field: an agent shipped "cooperative parallel LCP" as the cause of a slowdown; it was *wrong* per
  `objects.h:175` — the LCP is single-threaded. The measurements were fine; the *explanation* was unchecked.)
- **A parser over your own output is unverified code.** Check derived numbers against the raw log. (Field: a
  regex silently dropped the fastest timings and produced a wrong speedup table; only re-reading the raw log
  caught it.)

## 2. Hypothesis-refutation as a first-class result

- Treat each hypothesis as **falsifiable**, and a *refuted* one that narrows the cause as a real result.
  (Field: 4 refutation cycles, each ruling out a config, until "the drift was *identical* across every other
  setting" pinned the source to parallel island iteration.)
- **Sweep one parameter with a negative control**; **ablate one lever at a time** to attribute cause. Report
  the boundary (where behavior flips), not a single point.

## 3. Diagnosing a misbehaving sim

- **Reproduce the exact symptom HONESTLY first.** If the "broken" config behaves fine on defaults, *parameterize
  to find the real threshold* — never fudge the config until it looks broken. (Field: "the broken config did
  not actually misbehave… that is an honest result" → the agent found the true threshold instead.)
- **Most "the physics looks wrong" is solver convergence, not a wrong API call.** The classic symptoms —
  **sink, jitter, roll-forever, limb-stretch** — are signatures of *which* stepper you chose and how you tuned
  it. Ablate the solver levers to attribute cause: iterations-alone (fixes geometric sink, burns CPU), soft
  contacts alone (kills jitter but supplies no constraint force → still sinks), the *full* fix (iterations +
  soft CFM/ERP + contact surface layer + auto-disable) → jitter exactly 0. See
  `references/foundations/stepping-and-stability.md`, `erp-cfm-friction.md`, and `references/pitfalls.md`.
- **Sharpen a confounded metric.** When one number can be moved by two different causes, split it (e.g.
  floor-sink vs stack-compression vs adjacent-overlap). A single global metric hides the real cause.

## 4. The separate-context review pass — do NOT self-approve

Authoring and the correctness audit are **separate passes**. After you build and self-check, get an
*independent* review:

- Run a **read-only reviewer in a fresh context**. Give it the **claim and the headers, never your own
  reasoning** — a verifier that inherits your context mirrors your summary instead of recomputing (the
  independent-oracle rule in `verifying-simulations.md`).
- **It is load-bearing, not ceremony.** In two field sessions a separate-context reviewer found bugs the
  author had *not* caught: a wrong causal claim (corrected against `objects.h:175`) and **two harness bugs** —
  a near-tautological determinism check and a falsification that *passed on gravity alone*. The author's own
  lesson: *"the review changed the harness, not just the prose."*
- **Re-check a reviewer's surprising correction yourself** against the source before accepting it; don't relay
  a sub-agent's claim unverified.

## Anti-patterns (each cost a real field cycle)

| Anti-pattern | Do instead |
|---|---|
| A mechanism explanation that "sounds right" but was never checked against the header | Cite the header line; do an independent read |
| Trusting self-written parser/script numbers | Check against the raw data; a script that can't FAIL isn't a check, and one that errors should fail loud |
| "It built headless, so it's correct" | A clean headless build does NOT certify the graphical build — each `#ifdef` arm preprocesses the other away; compile **both** modes `-Wall -Wextra` |
| Porting a demo but changing its physics | Faithful-port: match the original's gravity/step/params exactly, then verify against the known behavior |
| Self-approving your own verdict | Separate-context review (§4) |

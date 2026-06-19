# Tokens: the tunable parameters of an ODE simulation

> In a UI design system, *tokens* are the named scalar values (color, spacing, radius) that every component reads. ODE's analog is its **tunable physics parameters** — the scalars that decide how the whole world *feels* and whether it is stable. Set them deliberately, like tokens, instead of leaving defaults implicit. Every value below is grounded in `include/ode/objects.h`, `contact.h`, or `common.h`. **The header default wins; the manual's recommended ranges are guidance.** Tuning recipes live in `references/foundations/stepping-and-stability.md`; the meaning of ERP/CFM/friction in `references/foundations/erp-cfm-friction.md`.

## World tokens (global — set once on the `dWorldID`)

| Token | Default | Range / guidance | Setter | Source |
|---|---|---|---|---|
| Gravity | `(0,0,0)` | set explicitly, e.g. `(0,0,-9.81)` | `dWorldSetGravity` | `objects.h:93` |
| **ERP** | `0.2` | `0.1–0.8`; avoid 0 (drift) and 1 (unstable) | `dWorldSetERP` | `objects.h:110` |
| **CFM** | `1e-5` single / `1e-10` double | `1e-9 … 1`; never negative | `dWorldSetCFM` | `objects.h:127` |
| QuickStep iterations | `20` (`dWORLDQUICKSTEP_ITERATION_COUNT_DEFAULT`) | raise for accuracy on big systems | `dWorldSetQuickStepNumIterations` | `objects.h:462` (default `:451`) |
| QuickStep over-relaxation `W` | (default lives in `ode/src`, not the header; ~1.3 per the manual) [VERIFY] | tuning knob for the iterative solver | `dWorldSetQuickStepW` | `objects.h:584` |
| Contact max correcting velocity | `dInfinity` | lower it to stop deep objects "popping" | `dWorldSetContactMaxCorrectingVel` | `objects.h:603` |
| Contact surface layer | `0` | `~0.001` reduces resting jitter | `dWorldSetContactSurfaceLayer` | `objects.h:623` |
| Linear damping (scale) | `0` | `[0,1]` | `dWorldSetLinearDamping` | `objects.h:826` |
| Angular damping (scale) | `0` | `[0,1]` | `dWorldSetAngularDamping` | `objects.h:840` |
| Linear damping threshold | — | speed below which damping is skipped | `dWorldSetLinearDampingThreshold` | `objects.h:798` |
| Angular damping threshold | — | as above (angular) | `dWorldSetAngularDampingThreshold` | `objects.h:812` |
| Max angular speed | `dInfinity` | clamp (keep `<500` if gyroscopic term on) | `dWorldSetMaxAngularSpeed` | `objects.h:865` |
| Auto-disable flag | `false` (off) | enable so resting bodies sleep | `dWorldSetAutoDisableFlag` | `objects.h:748` |
| Auto-disable linear threshold | `0.01` | idle linear-speed cutoff | `dWorldSetAutoDisableLinearThreshold` | `objects.h:677` |
| Auto-disable angular threshold | `0.01` | idle angular-speed cutoff | `dWorldSetAutoDisableAngularThreshold` | `objects.h:691` |
| Auto-disable steps | `10` | idle steps before sleeping | `dWorldSetAutoDisableSteps` | `objects.h:720` |
| Auto-disable time | `0` | idle seconds before sleeping | `dWorldSetAutoDisableTime` | `objects.h:734` |
| Auto-disable avg samples | `1` | **0 disables auto-disable entirely** | `dWorldSetAutoDisableAverageSamplesCount` | `objects.h:706` |

> **Step size is a per-call token, not stored.** There is no `dWorldSetTimestep` — you pass `stepsize` to `dWorldStep`/`dWorldQuickStep` every call (`objects.h:384,427`). **Keep it small and FIXED** (a variable step destabilizes ERP/CFM-tuned contacts). Damping/auto-disable world tokens are inherited by bodies *at creation time*; per-body overrides exist (`references/bodies-and-mass.md`).

## Contact (surface) tokens — per contact, in `dContact.surface` (`contact.h`)

`mode` (the bitmask) and `mu` are **always** read; every other field is read only if its `dContact*` flag is OR'd into `mode` (see `references/foundations/erp-cfm-friction.md`).

| Token (field) | Meaning | Gate flag | Source |
|---|---|---|---|
| `mu` | friction coeff/limit, dir 1 — `0`…`dInfinity` (always set) | — | `contact.h:58` |
| `mu2` | friction coeff, dir 2 (anisotropic) | `dContactMu2` (0x001) | `contact.h:34` |
| `bounce`, `bounce_vel` | restitution + min impact speed to bounce | `dContactBounce` (0x004) | `contact.h:37` |
| `soft_erp` | per-contact ERP (overrides global) | `dContactSoftERP` (0x008) | `contact.h:38` |
| `soft_cfm` | per-contact CFM (compliant contact) | `dContactSoftCFM` (0x010) | `contact.h:39` |
| `motion1/2/N` | surface (conveyor) velocity | `dContactMotion1/2/N` (0x020/40/80) | `contact.h:40-42` |
| `slip1/slip2` | force-dependent slip | `dContactSlip1/Slip2` (0x100/0x200) | `contact.h:43-44` |
| `rho/rho2/rhoN` | rolling / spinning friction | `dContactRolling` (0x400) | `contact.h:45` |
| (friction model) | box (force limit) vs pyramid (Coulomb ratio) | `dContactApprox1` (0x7000) | `contact.h:51` |

Contacts have **no library defaults** — you fill the struct every step in the near-callback. Typical demo values: `soft_cfm ~0.01–0.3`, `soft_erp ~0.5–0.96`, `bounce ~0.1–0.2` (these are demo choices, not defaults; see `references/collision-and-contacts.md`).

## Joint tokens — limits & motors, via `dJointSet<Type>Param(j, dParamX, value)` (`common.h`)

The full `dParam` family table (`dParamLoStop`/`dParamHiStop`/`dParamVel`/`dParamFMax`/`dParamFudgeFactor`/`dParamBounce`/`dParamCFM`/`dParamStop{ERP,CFM}`/`dParamSuspension{ERP,CFM}`/`dParamGroup`) with `common.h` lines lives in **`references/joints.md` → "dParam family"** — single owner; not duplicated here. Key rule: a motor only acts when `dParamFMax > 0` (`common.h:446`). To move a stop reliably set **hi, then lo, then hi** again (FAQ).

## Using tokens well

- Pick **ERP/CFM** first (they set the global "give" of every constraint), then add **soft_cfm/soft_erp** on resting/stacked contacts for stability.
- Enable **auto-disable** + use **`dWorldQuickStep`** for large stacks.
- A small **surface layer** (`~0.001`) + a finite **max correcting velocity** tame jitter and "popping".
- Keep the **step size** small and fixed; scale your world so lengths/masses are roughly `0.1–1.0` for single-precision factorizer precision (FAQ).

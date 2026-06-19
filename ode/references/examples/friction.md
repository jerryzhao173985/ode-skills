# Example: friction — the Coulomb slide/stick threshold

> Source: `ode/demo/demo_friction.cpp` (205 lines). Verified line anchors. The clearest demo of **where and how friction is configured** (`surface.mu`, `dContactApprox1`).

## What it builds
A grid of boxes on a plane, each pushed horizontally; mass varies down one axis and push force across the other, so the slide/stick boundary (force > μ·m·g) forms a visible triangle.

## Components composed
- **World** (`dWorldSetGravity`) + **Space** + contact **JointGroup** + ground **Geom** (`dCreatePlane`).
- A grid of **Body**+**Mass**+**Geom** boxes (`dGeomSetBody`).
- **Contact** joints whose `surface` tokens are the whole point.

## Key calls (walkthrough)
| Line | Call | Role |
|---|---|---|
| 105 | `contact[i].surface.mode = dContactSoftCFM \| dContactApprox1` | pick the friction model |
| 106 | `contact[i].surface.mu = MU` | **the** friction coefficient (per contact) |
| 107 | `contact[i].surface.soft_cfm = 0.01` | compliant contact |
| 109 | `dCollide(o1,o2,3,&contact[0].geom,sizeof(dContact))` | narrow-phase |
| 111–112 | `dJointCreateContact` + `dJointAttach` | build contact joints |
| 139 | `dBodyAddForce(body[i][j],FORCE*(i+1),0,0)` | per-step push (varies across grid) |
| 143–147 | `dSpaceCollide` → `dWorldStep(world,0.05)` → `dJointGroupEmpty` | the loop |
| 180–181 | `dWorldSetGravity(world,0,0,-GRAVITY)` + `dCreatePlane` | normal force source + floor |

## Patterns to copy
- **Friction lives on the contact**, set in the near-callback every step — *not* on the body or geom. `surface.mu` only (isotropic); add `dContactMu2`+`mu2` for anisotropic.
- **`dContactApprox1` is essential** — it makes friction scale with the real normal force (the physical Coulomb model). Without it the clean slide-triangle wouldn't form (and rolling bodies can lock — FAQ).
- Configure `surface` **before** `dCollide` (it writes into `&contact[0].geom` with `sizeof(dContact)` stride, leaving your surface fields intact).

## Gotchas
- No persistent friction setting exists anywhere — it is rebuilt per contact, per step.
- The XOR filter `if (!(g1 ^ g2)) return;` processes only box-vs-ground (boxes ignore each other).
- Forgetting `dJointGroupEmpty` leaks/accumulates contacts.

## See also
`references/foundations/erp-cfm-friction.md` (the friction model + box vs pyramid), `references/collision-and-contacts.md`, `references/tokens.md` (surface tokens).

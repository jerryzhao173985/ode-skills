# Units and Scaling

ODE is a numerical solver: it factorizes constraint matrices in floating point.
Extreme magnitudes (tiny or huge lengths/masses) burn through the precision the
factorizer needs and surface as jitter or "explosions." This file is about keeping
the numbers in a stable window.

There is no enforced unit system — ODE is unit-agnostic. The wiki recommends MKS/SI
consistency and a magnitude window; it does not mandate a specific unit system.
Cite: `https://www.ode.org/wiki/index.php?title=FAQ (Should I scale my units to be around 1.0?)`.

---

## The magnitude window

Keep object SIZES roughly between 0.1 and 10, and MASSES within a few orders of
magnitude (ideally 0.1..1.0). Cite: `https://www.ode.org/wiki/index.php?title=FAQ
(Should I scale my units to be around 1.0?)`. The 0.1..1.0 band is best for
factorizer precision, especially in single precision.

If you must simulate very small (mm / grams) or very large (km / tonnes) quantities
and hit factorizer-precision instability, scale lengths and masses to ~0.1..10 and
scale the time step accordingly — the same advice applies to very large OR very
small systems. Cite: `https://www.ode.org/wiki/index.php?title=FAQ (Should I scale
my units to be around 1.0?)`.

When you rescale lengths/masses for stability, scale gravity (and the time step)
accordingly so the dynamics stay consistent. Cite: `include/ode/objects.h:93`
(`dWorldSetGravity`).

```c
void dWorldSetGravity (dWorldID, dReal x, dReal y, dReal z);
```
Cite: `include/ode/objects.h:93`.

Prefer double precision when stability matters; single precision gives more
numerical error and less stability. Cite: `https://www.ode.org/wiki/index.php?title=FAQ
(Should I scale my units to be around 1.0?)`. This is also why the default global
CFM differs by precision: 1e-5 (single) vs 1e-10 (double). Cite:
`include/ode/objects.h:120-127`.

---

## Mass ratios

Avoid extreme MASS RATIOS between jointed bodies (the "whip" problem): joints
connecting large and small masses have a harder time keeping error low. Cite:
`https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Making things
stable)`. The constraint matrix `J*inv(M)*J'` becomes ill-conditioned and
QuickStep's PGS solver diverges (explosions). Keep contact CFM near 0 and ERP ~0.2
on wheel-ground contacts, or split heavy bodies. Cite:
`https://classic.gazebosim.org/tutorials?tut=physics_params`.

Avoid extreme SIZE RATIOS in collision too: a size-1 sphere resting on a 1000-unit
polygon jitters and bounces while resting — subdivide the large polygon. Cite:
`https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Making things
stable)`.

---

## Density realism: the sub-air-density tunneling trap

Set a realistic total mass (e.g. `dMassSetBoxTotal` or `dMassAdjust`), not a tiny
density. A default density of 1 kg/m^3 is less dense than air, so bodies barely
react and appear to pass through each other. Cite:
`https://www.gamedev.net/forums/topic/571803-ode-collision-joint-problems/`.

```c
void dMassSetBoxTotal (dMass *, dReal total_mass, dReal lx, dReal ly, dReal lz);
void dMassAdjust (dMass *, dReal newmass);
```
Cite: `include/ode/mass.h:67, 74`.

```c
dMass m;
dMassSetBoxTotal(&m, 1.0 /*kg*/, sx, sy, sz);  /* real total mass */
dBodySetMass(body, &m);                         /* not density=1 */
```
Cite: `https://www.gamedev.net/forums/topic/571803-ode-collision-joint-problems/`.

`dMassSetBox` takes a DENSITY; `dMassSetBoxTotal` takes a TOTAL MASS. Cite:
`include/ode/mass.h:65, 67`. When in doubt, set a real total mass so contact forces
can actually move the body.

---

## Step-size scaling (related)

Step composition is non-associative: 10 steps of 0.1 != 5 steps of 0.2. Smaller
step = more accurate AND more stable. Tune all parameters (ERP/CFM/contacts) at the
final/real frame rate you will ship with. Cite:
`https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Behavior depends
on step size)`. When you rescale the world, rescale the time step too. Cite:
`https://www.ode.org/wiki/index.php?title=FAQ (Should I scale my units to be around 1.0?)`.

(Full fixed-vs-variable step-size guidance lives in the world/stepping reference;
here the point is only that the time step is part of the unit-scaling bundle.)

---

## Rules

- Keep object SIZES roughly 0.1..10 and MASSES within a few orders of magnitude
  (ideally 0.1..1.0). Cite: `https://www.ode.org/wiki/index.php?title=FAQ (Should I
  scale my units to be around 1.0?)`.
- ODE has no enforced units; keep them MKS/SI-consistent. The window is a magnitude
  recommendation, not a mandated unit system. Cite: `https://www.ode.org/wiki/index.php?title=FAQ (Should I scale my units to be around 1.0?)`.
- When you rescale lengths/masses, rescale gravity and the time step too. Cite:
  `include/ode/objects.h:93`, `https://www.ode.org/wiki/index.php?title=FAQ`.
- Avoid extreme mass ratios between jointed bodies (the whip problem) and extreme
  size ratios in collision (subdivide huge polygons). Cite: `https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Making things stable)`.
- Set a realistic total mass with `dMassSetBoxTotal`/`dMassAdjust`; density 1
  (kg/m^3) is lighter than air and bodies will appear to pass through each other.
  Cite: `https://www.gamedev.net/forums/topic/571803-ode-collision-joint-problems/`,
  `include/ode/mass.h:67, 74`.
- Prefer double precision when stability matters; single precision has more error.
  Cite: `https://www.ode.org/wiki/index.php?title=FAQ`.
- Smaller, fixed steps are more stable; steps are non-associative; tune at your ship
  frame rate. Cite: `https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Behavior depends on step size)`.

---

## Common mistakes

| Bad | Good | Why |
|-----|------|-----|
| Simulating raw units like a 0.005 m / 0.002 kg part or a 2000 m / 50000 kg vehicle, then fighting jitter/explosions | Scale the system so lengths/masses sit ~0.1..10 (ideally 0.1..1.0), and scale gravity and the time step to match | Tiny/huge magnitudes lose precision in the LCP factorizer (worse in single precision), surfacing as instability; rescaling to ~1.0 restores headroom. Cite: `https://www.ode.org/wiki/index.php?title=FAQ (Should I scale my units to be around 1.0?)` |
| `dMassSetBox(&m, 1.0, sx, sy, sz)` — density 1 kg/m^3 — expecting solid collisions | `dMassSetBoxTotal(&m, 1.0 /*kg*/, sx, sy, sz); dBodySetMass(body, &m);` | Density 1 kg/m^3 is lighter than air, so contact forces barely move the body and objects appear to pass through. Cite: `https://www.gamedev.net/forums/topic/571803-ode-collision-joint-problems/` |
| Connecting a heavy body directly to a very light one (heavy chassis on light wheels) and expecting stable joints | Keep jointed masses within a few orders of magnitude; merge tiny bodies, use kinematic fakes, or raise global CFM | "Joints that connect large and small masses together will have a harder time keeping their error low" — ill-conditioned `J*inv(M)*J'` diverges in PGS. Cite: `https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Making things stable)`, `https://classic.gazebosim.org/tutorials?tut=physics_params` |
| A size-1 sphere resting on a single 1000-unit ground polygon | Subdivide the large polygon (or use `dCreatePlane` for an infinite ground) | Extreme size ratios make the resting sphere jitter/bounce. Cite: `https://ode.org/wiki/index.php/HOWTO_make_the_simulation_better (Making things stable)` |
| Rescaling the world for stability but leaving gravity and the time step unchanged | Scale gravity and `dt` by the same scaling so dynamics stay consistent | Dynamics depend on length/mass/time together; rescaling only lengths breaks the equations. Cite: `https://www.ode.org/wiki/index.php?title=FAQ`, `include/ode/objects.h:93` |

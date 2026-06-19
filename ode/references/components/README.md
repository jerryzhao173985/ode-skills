# Components: the ODE object catalog

> Think of ODE as a **design system**: the objects below are the *components* you instantiate and compose;
> the scalar parameters in `references/tokens.md` are the *design tokens* they read; the rules in
> `references/foundations/` are the *principles*. This dir holds a focused **card per concrete element**
> (`joint-<type>.md`, `geom-<type>.md`); the object-level overview API lives in the core references named below.

## The objects (overview API)

| Object | Handle | One-liner | Overview reference |
|---|---|---|---|
| **World** | `dWorldID` | dynamics container + global solver tokens | `references/world-and-stepping.md` |
| **Body** | `dBodyID` | rigid body: mass + motion, **no shape** | `references/bodies-and-mass.md` |
| **Mass** | `dMass` (struct) | a body's mass/inertia, built then attached | `references/bodies-and-mass.md` |
| **Space** | `dSpaceID` | broadphase container for geoms | `references/geoms-and-spaces.md` |
| **Geom** | `dGeomID` | a collision shape, **no mass** | `references/geoms-and-spaces.md` |
| **Joint** | `dJointID` | a constraint between two bodies (incl. motors) | `references/joints.md` |
| **Contact** | `dContact` + contact `dJointID` | the per-step bridge from collision to dynamics | `references/collision-and-contacts.md` |

## The per-element cards (this directory)

- **Joints** — `joint-ball`, `joint-hinge`, `joint-slider`, `joint-universal`, `joint-fixed`,
  `joint-hinge2`, `joint-piston`, `joint-pr`, `joint-pu`, `joint-plane2d`, `joint-amotor`, `joint-lmotor`,
  `joint-dball`, `joint-dhinge`, `joint-transmission`, `joint-null` (`joint-<type>.md`).
- **Geoms** — `geom-sphere`, `geom-box`, `geom-capsule`, `geom-cylinder`, `geom-plane`, `geom-ray`,
  `geom-convex`, `geom-heightfield`, `geom-trimesh` (`geom-<type>.md`). Trimesh/heightfield depth:
  `references/trimesh-heightfield.md`.

## How components compose

```
                         dWorldID (World)  ───── reads ──▶  tokens: gravity, ERP, CFM, …
                          │  owns                                   (references/tokens.md)
            ┌─────────────┼──────────────────┐
            ▼             ▼                  ▼
        dBodyID       dJointID            dJointGroupID
        (Body)        (Joint)             (holds contact joints, emptied each step)
          │  dBodySetMass(&dMass)            ▲
          │                                  │ dJointCreateContact + dJointAttach
          │  dGeomSetBody(geom, body)        │ (one per dCollide hit, every step)
          ▼   ◀──── the crux link ────▶      │
        dGeomID ───── lives in ─────▶ dSpaceID (Space) ──▶ dSpaceCollide → nearCallback → dCollide
        (Geom: shape)                 (broadphase)            (produces dContact points)
```

**The wiring rules, in one breath:** a *World* owns *Bodies* and *Joints*; a *Space* owns *Geoms*; a *Body*
gets its mass from a *Mass* (`dBodySetMass`) and its shape from a *Geom* (`dGeomSetBody`); a *Joint* attaches
two *Bodies* (or a body + the static world via body `0`); each step, collision turns overlaps into *Contact*
joints placed in a group that is emptied after the step. The full ordered loop is the canonical example in
`SKILL.md`; a runnable composition is `references/examples/boxstack.md`.

## Reading order for a newcomer

1. `references/foundations/mental-model.md` — the mental model (body≠geom, the loop).
2. `references/world-and-stepping.md` → `references/bodies-and-mass.md` → `references/geoms-and-spaces.md` — instantiate the pieces.
3. `references/collision-and-contacts.md` — wire collision into dynamics.
4. `references/joints.md` (+ the relevant `joint-<type>.md` card) — connect bodies.
5. `references/tokens.md` + `references/foundations/stepping-and-stability.md` — make it stable.
6. `references/foundations/verifying-simulations.md` — prove it actually works.

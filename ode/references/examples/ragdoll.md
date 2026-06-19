# Example: rag-doll — a tree of bodies joined with limited joints

> Source: wiki **Rag-doll Character** (tips) + `ode/demo/demo_joints.cpp` + headers. The wiki page is a stub (links out, no code), so the architecture below is grounded in the local headers/demos following the wiki's tips. The canonical **articulated-character** composition.

## What it builds
A character as capsule limb **Bodies** connected by joints whose `dParamLoStop/HiStop` limits model anatomical ranges; jointed limbs skip mutual collision.

## Components composed
Per limb: **Body** + capsule **Geom** (`dCreateCapsule`) + **Mass** (`dMassSetCapsule`). Between limbs: **Joints** chosen per anatomy. Self-collision filtered in the near-callback with `dAreConnected`.

## Joint → anatomy mapping
| Anatomical joint | ODE joint | Create | DOF | Limits |
|---|---|---|---|---|
| shoulder, hip | ball | `dJointCreateBall` (`objects.h:1693`) | 3 | **no built-in stops** → add an AMotor (`dJointCreateAMotor` `objects.h:1776`) |
| elbow, knee | hinge | `dJointCreateHinge` (`objects.h:1701`) | 1 | `dParamLoStop`/`dParamHiStop` |
| neck, wrist, ankle | universal | `dJointCreateUniversal` (`objects.h:1733`) | 2 | `dParam…Stop` + `…Stop2` |

(Wiki tips: use **capsules** for limbs — fewer awkward collisions; prefer ball/universal over a single hinge for shoulders/hips — a 1-DOF shoulder is "unnatural"; auto-disable the ragdoll **as one island**, not per limb.)

## A limited hinge limb (from `demo_joints.cpp`)
```c
dJointID j = dJointCreateHinge(world, 0);
dJointAttach(j, parentBody, childBody);          // position bodies FIRST (anchors are global)
dJointSetHingeAnchor(j, ax, ay, az);             // objects.h:1983 — world-space pivot
dJointSetHingeAxis(j, 0, 0, 1);                  // objects.h:1991
dJointSetHingeParam(j, dParamLoStop, 0.0);       // e.g. elbow can't hyperextend
dJointSetHingeParam(j, dParamHiStop, 2.4);       // ~135° (radians)
dJointSetHingeParam(j, dParamFMax, 1);           // needed for a motorized/limited limb
```
A ball shoulder uses `dJointSetBallAnchor(j, x,y,z)` (`objects.h:1965`) and an AMotor for limits; a universal uses `dJointSetUniversalAnchor` (`objects.h:2124`) + `dJointSetUniversalAxis1/2` (`objects.h:2130,2175`).

## Skip self-collision between jointed limbs
```c
static void nearCallback(void *d, dGeomID o1, dGeomID o2) {
    dBodyID b1 = dGeomGetBody(o1), b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1, b2)) return;   // objects.h:3484 — don't fight the joint
    /* … dCollide → dJointCreateContact → dJointAttach … */
}
```

## Assembly recipe
1. `dInitODE2(0)`; create world + space.
2. Per limb: body → position → capsule geom → `dMassSetCapsule`(`mass.h:55`, `direction` 1=x/2=y/3=z = long axis) + `dBodySetMass` → `dGeomSetBody`.
3. Connect adjacent limbs: create joint → `dJointAttach(j, parent, child)` → set anchor/axis → set `dParamLoStop/HiStop`.
4. Skip collision for jointed pairs (`dAreConnected`); auto-disable the whole island.
5. Standard per-step loop (`references/world-and-stepping.md`).

## Gotchas
- **Position bodies before attaching joints** — anchors/axes are global, and setting a hinge anchor/axis **resets the joint's zero angle** (`objects.h` hinge docs).
- **Ball joints have no stops** — limit a shoulder/hip with an AMotor, not the ball joint itself.
- Stops are **radians** for rotational joints, meters for sliders.
- Hinge limits near ±π behave wrong ([known-issues #84]); set a stop with hi → lo → hi (FAQ).

## See also
`references/joints.md` (all joint types + `dParam*`), `references/components/{body,joint,mass}.md`, `references/foundations/recipes.md` (ragdoll recipe), `references/foundations/known-issues.md`.

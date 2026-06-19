# Joint: Ball-and-socket  (`dJointCreateBall`)

> Source of truth: include/ode/objects.h:1693. Cited by file:line; do not invent symbols.

## What it is

A ball-and-socket joint constrains a single anchor **point** on body1 to coincide with the same point on body2; it removes the 3 translational DOF and leaves all **3 rotational DOF** free (the bodies can pivot freely about the anchor). The anchor is set once in world coordinates (`include/ode/objects.h:1965`).

## Create & attach

```c
dJointID joint = dJointCreateBall (world, 0);      /* group 0 = allocate normally */
dJointAttach (joint, body1, body2);                /* REQUIRED; 0 for a body => static env */
dJointSetBallAnchor (joint, x, y, z);              /* anchor in WORLD coords */
```

Verbatim shape from `ode/demo/demo_chain1.c:157-160`:

```c
joint[i] = dJointCreateBall (world,0);
dJointAttach (joint[i],body[i],body[i+1]);
dJointSetBallAnchor (joint[i],k,k,k+0.4);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetBallAnchor` | `void dJointSetBallAnchor (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1965 | Set the ball anchor in world coordinates. |
| `dJointSetBallAnchor2` | `void dJointSetBallAnchor2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:1971 | Set the second anchor (body2 side) in world coordinates. |
| `dJointGetBallAnchor` | `void dJointGetBallAnchor (dJointID, dVector3 result)` | include/ode/objects.h:2555 | Read the anchor as seen on body1 (writes into `result`). |
| `dJointGetBallAnchor2` | `void dJointGetBallAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:2567 | Read the anchor as seen on body2; compare with `dJointGetBallAnchor` to see how far the joint has come apart. |

## Parameters / conventions

- **Anchor**: world coordinates, set after attach. Because the joint is purely point-to-point, there is **no axis** and **no angle**.
- **dParam* family**: the ball joint has no limits or motor of its own (no `dJointSetBallParam` exists — see "Never invent"). To motorize or limit rotation about a ball joint, attach a separate AMotor.
- Per-body anchor readback: `dJointGetBallAnchor` returns the point on body1, `dJointGetBallAnchor2` the point on body2; identical (to roundoff) when the constraint is satisfied (`include/ode/objects.h:2561-2565`).

## Pitfalls

- Forgetting `dJointAttach` leaves the joint in "limbo" with zero effect on the simulation (`include/ode/objects.h:1688`).
- `dJointAttach(joint, b1, b2)`: passing 0 for one body anchors to the static environment; passing 0 for both leaves it inert (`include/ode/objects.h:1866`).
- A ball joint allocated in a joint group cannot be freed by `dJointDestroy`; empty/destroy the group instead (`include/ode/objects.h:1824`).

## Never invent

- `dJointSetBallParam` / `dJointGetBallParam` — the ball joint exposes no dParam setter (use an AMotor for limits/motors).
- `dJointSetBallAxis` / `dJointGetBallAngle` — a ball joint has no axis or angle.

See also the joints overview: `references/joints.md`.

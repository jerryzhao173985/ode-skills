# Joint: Double Hinge (`dJointCreateDHinge`)

> Source of truth: include/ode/objects.h:1808. Cited by file:line; do not invent symbols.

## What it is

A double-hinge joint couples two bodies through a single shared axis plus two anchor points (one per body), constraining their relative motion to a hinge-like behavior at a set separation along that axis. Cited by include/ode/objects.h:1808, include/ode/objects.h:3415, include/ode/objects.h:3451.

## Create & attach

```c
dJointID j = dJointCreateDHinge(world, 0);    /* group 0 = normal alloc */
dJointAttach(j, body1, body2);                /* required; 0 for a body = static env */
dJointSetDHingeAxis(j, 0, 0, 1);              /* shared axis, world coords */
dJointSetDHingeAnchor1(j, x1, y1, z1);        /* anchor on body1, world coords */
dJointSetDHingeAnchor2(j, x2, y2, z2);        /* anchor on body2, world coords */
```

Cited by include/ode/objects.h:1808, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:3415 (axis), include/ode/objects.h:3427 (anchor1), include/ode/objects.h:3433 (anchor2).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetDHingeAxis` | `void dJointSetDHingeAxis (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3415 | Set the shared hinge axis (world coords). |
| `dJointGetDHingeAxis` | `void dJointGetDHingeAxis (dJointID, dVector3 result)` | include/ode/objects.h:3421 | Get the axis into `result`. |
| `dJointSetDHingeAnchor1` | `void dJointSetDHingeAnchor1 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3427 | Set anchor1 (on body1). |
| `dJointSetDHingeAnchor2` | `void dJointSetDHingeAnchor2 (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:3433 | Set anchor2 (on body2). |
| `dJointGetDHingeAnchor1` | `void dJointGetDHingeAnchor1 (dJointID, dVector3 result)` | include/ode/objects.h:3439 | Get anchor1 into `result`. |
| `dJointGetDHingeAnchor2` | `void dJointGetDHingeAnchor2 (dJointID, dVector3 result)` | include/ode/objects.h:3445 | Get anchor2 into `result`. |
| `dJointGetDHingeDistance` | `dReal dJointGetDHingeDistance (dJointID)` | include/ode/objects.h:3451 | Get the set distance between anchors. |
| `dJointSetDHingeParam` | `void dJointSetDHingeParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:3457 | Set a double-hinge `dParam*` parameter. |
| `dJointGetDHingeParam` | `dReal dJointGetDHingeParam (dJointID, int parameter)` | include/ode/objects.h:3463 | Read back a double-hinge `dParam*` value. |

## Parameters / conventions

- A single shared axis is set via `dJointSetDHingeAxis` in world coordinates (unlike the double-ball joint, which has no axis). Cited by include/ode/objects.h:3412, include/ode/objects.h:3364.
- Two anchors, one per body, set in world coordinates via `dJointSetDHingeAnchor1`/`Anchor2`. Cited by include/ode/objects.h:3424, include/ode/objects.h:3430.
- The separation between anchors is read back with `dJointGetDHingeDistance`; constraint tuning goes through `dJointSetDHingeParam`. Cited by include/ode/objects.h:3448, include/ode/objects.h:3454.

## Pitfalls

- Set axis and anchors *after* attaching; an unattached joint has no effect. Cited by include/ode/objects.h:1688.
- There is a getter (`dJointGetDHingeDistance`) but no `dJointSetDHingeDistance` — the distance is derived from the anchors, not set directly. Cited by include/ode/objects.h:3451.
- Use `dJointSetDHingeAnchor1`/`Anchor2`, not a single un-numbered anchor setter. Cited by include/ode/objects.h:3427.

## Never invent

- `dJointSetDHingeDistance` — no such setter; only `dJointGetDHingeDistance` exists. Cited by include/ode/objects.h:3451.
- `dJointSetDHingeAxis2` — the double-hinge has only one shared axis. Cited by include/ode/objects.h:3415.
- `dJointGetDHingeAngle` — no angle getter exists for this joint in include/ode/objects.h. Cited by include/ode/objects.h:3463.

Cross-link: references/joints.md (joints overview).

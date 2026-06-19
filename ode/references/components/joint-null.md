# Joint: Null (`dJointCreateNull`)

> Source of truth: include/ode/objects.h:1768. Cited by file:line; do not invent symbols.

## What it is

A null joint imposes no constraint (DOF removed = 0). It is a placeholder you can attach to two bodies as a structural slot — useful for runtime-swappable joint logic — without affecting the simulation. Cited by include/ode/objects.h:1768.

## Create & attach

```c
dJointID j = dJointCreateNull(world, 0);   /* group 0 = normal alloc */
dJointAttach(j, body1, body2);             /* required; 0 for a body = static env */
/* no axis/anchor/param setters exist for a null joint */
```

Cited by include/ode/objects.h:1768, include/ode/objects.h:1873 (dJointAttach), include/ode/objects.h:1763 (group arg: 0 = normal alloc, nonzero = joint group).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointCreateNull` | `dJointID dJointCreateNull (dWorldID, dJointGroupID)` | include/ode/objects.h:1768 | Create a null joint (no constraint). |

The null joint has no element-specific setters or getters in include/ode/objects.h; only the generic joint functions (`dJointAttach`, `dJointGetType` returning `dJointTypeNull`, `dJointDestroy`, etc.) apply. Cited by include/ode/objects.h:1768, include/ode/objects.h:1921 (`dJointTypeNull`).

## Parameters / conventions

- No `dParam*` parameters apply — a null joint has nothing to limit or motorize. Cited by include/ode/objects.h:1768.
- `dJointGetType` on a null joint returns `dJointTypeNull`. Cited by include/ode/objects.h:1921.
- The `dJointGroupID` argument follows the usual rule: 0 allocates normally, nonzero allocates in that joint group. Cited by include/ode/objects.h:1763.

## Pitfalls

- Even with no constraint, the joint occupies the world's joint list; destroy it like any other joint (or via its group). Cited by include/ode/objects.h:1827.
- Do not look for axis/anchor/param setters — none exist for this type. Cited by include/ode/objects.h:1768.

## Never invent

- `dJointSetNullParam` / `dJointSetNullAxis` — no such functions; the null joint has no configurable state. Cited by include/ode/objects.h:1768.

Cross-link: references/joints.md (joints overview).

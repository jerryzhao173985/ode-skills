# Geom: Box  (`dCreateBox`)

> Source of truth: include/ode/collision.h:1001. Cited by file:line; do not invent symbols.

## What it is

A box geom defined by **FULL side lengths** `(lx, ly, lz)` along the X/Y/Z axes — these are the *total* extents, not half-extents. The reference point is its **center** (`include/ode/collision.h:1001`).

## Create & attach

```c
dGeomID box = dCreateBox (space, lx, ly, lz);   /* lx,ly,lz = FULL side lengths */
dGeomSetBody (box, body);                         /* bind to a body; 0 detaches */
```

Verbatim-style from the demo, where a box geom is attached to its body (`ode/demo/demo_buggy.cpp:237`):

```c
dGeomSetBody (box[0], body[0]);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeomBoxSetLengths` | `void dGeomBoxSetLengths (dGeomID box, dReal lx, dReal ly, dReal lz)` | include/ode/collision.h:1015 | Change the box's full side lengths. |
| `dGeomBoxGetLengths` | `void dGeomBoxGetLengths (dGeomID box, dVector3 result)` | include/ode/collision.h:1027 | Write the three full side lengths into `result`. |
| `dGeomBoxPointDepth` | `dReal dGeomBoxPointDepth (dGeomID box, dReal x, dReal y, dReal z)` | include/ode/collision.h:1042 | Depth of a point inside the box (>0 inside). |

## Parameters / conventions

- **Dimension**: FULL side lengths `lx, ly, lz`. A box of `dCreateBox(space, 2, 2, 2)` spans from `-1` to `+1` on each axis (half-extent is `lx/2`).
- **Get-dims function**: `dGeomBoxGetLengths` writes a `dVector3` of the three full lengths — it does **not** return a value (`include/ode/collision.h:1027`).
- **Reference point**: the box's center (`include/ode/collision.h:1001`).
- **Offsets**: displace from the body via `dGeomSetOffsetPosition` / `dGeomSetOffsetRotation` (`include/ode/collision.h:542`, `:558`).
- **Class**: `dGeomGetClass` returns `dBoxClass` (`include/ode/collision.h:307`, class ids at `:877`).

## Pitfalls

- **Full lengths, not half-extents**: ODE boxes take FULL side lengths in both `dCreateBox` and `dGeomBoxSetLengths` (`include/ode/collision.h:1015`). If you size the geom by half-extents it will be twice as large as intended; halve only when computing offsets/half-extents yourself.
- `dGeomBoxGetLengths` returns `void` and fills the passed `dVector3` — reading a return value is a bug (`include/ode/collision.h:1027`).
- A geom carries no mass; set the body's mass with the matching box mass helper separately, and bind with `dGeomSetBody` (`include/ode/collision.h:105`).

## Never invent

- `dGeomBoxGetSides` / `dGeomBoxSetSides` — the functions are `dGeomBoxGetLengths` / `dGeomBoxSetLengths`.
- `dGeomBoxGetExtents` / half-extent constructors — ODE uses full side lengths only.
- `dBoxCreate` — the constructor is `dCreateBox`.

---

See the overview: [geoms-and-spaces.md](../geoms-and-spaces.md).

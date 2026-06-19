# Geom: Plane  (`dCreatePlane`)

> Source of truth: include/ode/collision.h:1045. Cited by file:line; do not invent symbols.

## What it is

A **non-placeable**, infinite static plane defined by the equation `a*x + b*y + c*z = d`, where `(a,b,c)` is the plane normal (should be unit length) and `d` is the signed distance from the origin along that normal. It has **no body** and is positioned only by its four coefficients — typically used as the static ground (`include/ode/collision.h:1045`).

## Create & attach

A plane is non-placeable: do **not** give it a body, a position, or a rotation. The `a,b,c,d` coefficients are its complete pose.

```c
/* Ground plane: normal (0,0,1) = +Z up, passing through z=0. */
dGeomID ground = dCreatePlane (space, 0, 0, 1, 0);
/* No dGeomSetBody / dGeomSetPosition — the plane has no body and is not placeable. */
```

Verbatim shape from `ode/demo/demo_buggy.cpp:228`:

```c
ground = dCreatePlane (space,0,0,1,0);
```

In the collision `nearCallback`, a plane (like any static geom) returns body `0` from `dGeomGetBody`, which `dJointAttach` treats as the static environment (`ode/demo/demo_buggy.cpp:105`).

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dCreatePlane` | `dGeomID dCreatePlane (dSpaceID space, dReal a, dReal b, dReal c, dReal d)` | include/ode/collision.h:1045 | Create the non-placeable infinite plane `a*x+b*y+c*z=d`. |
| `dGeomPlaneSetParams` | `void dGeomPlaneSetParams (dGeomID plane, dReal a, dReal b, dReal c, dReal d)` | include/ode/collision.h:1046 | Re-set the plane coefficients (this is how you "move" a plane). |
| `dGeomPlaneGetParams` | `void dGeomPlaneGetParams (dGeomID plane, dVector4 result)` | include/ode/collision.h:1047 | Read back the four coefficients into a `dVector4`. |
| `dGeomPlanePointDepth` | `dReal dGeomPlanePointDepth (dGeomID plane, dReal x, dReal y, dReal z)` | include/ode/collision.h:1048 | Return the depth of point `(x,y,z)`; a point on the surface has depth zero (`include/ode/collision.h:1040`). |

## Parameters / conventions

- **Pose lives entirely in `(a,b,c,d)`.** There are no dimensions to query — the plane is infinite. To reposition or re-orient it, call `dGeomPlaneSetParams`, **not** `dGeomSetPosition`/`dGeomSetRotation`.
- **Normal convention**: `(a,b,c)` is the outward normal; `d` is the signed origin distance along it. A point on the surface has zero depth, points "above" the surface (on the normal side) have positive/negative depth per `dGeomPlanePointDepth` (`include/ode/collision.h:1040`).
- **No body, no offsets.** A plane is non-placeable, so it has no `dGeomSetBody`, no position, no rotation, and no `dGeomSetOffset*`.

## Pitfalls

- Calling `dGeomSetPosition`/`dGeomSetRotation` on a plane is a debug-build runtime error — planes are non-placeable; position/rotation setters require a placeable geom (`include/ode/collision.h:88`).
- To move the plane, set the coefficients: `dCreatePlane(space,0,0,1,5)` puts the ground at `z=5`, **not** `dGeomSetPosition(p,0,0,5)` (mistake noted in 04-collision.json mistakes).
- The plane is infinite and static; it has no body, so in the `nearCallback` `dGeomGetBody` returns `0` and the contact joint binds to the static environment (`ode/demo/demo_buggy.cpp:105`).

## Never invent

- `dGeomPlaneSetPosition` / `dGeomPlaneSetRotation` — a plane is non-placeable; use `dGeomPlaneSetParams`.
- `dGeomSetPlaneParams` — the function is `dGeomPlaneSetParams` (object-type prefix order).

See also the geoms overview: `references/geoms-and-spaces.md`.

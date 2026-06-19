# ODE C++ Wrapper Classes (odecpp.h, odecpp_collision.h)

> Source of truth: include/ode/odecpp.h, include/ode/odecpp_collision.h. Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- The C++ layer is a **thin, header-only RAII wrapper** over the C handle API: each wrapper just holds a `dXxxID` (`_id`) and forwards inline calls to the matching `dXxx*` C function. It exists only under `#ifdef __cplusplus` and adds no behavior beyond construct/destroy and convenience overloads (include/ode/odecpp.h:28).
- **You instantiate the typedef, not the template.** `dWorld`, `dBody`, `dHingeJoint`, `dSphere`, etc. are `typedef`s of the `*Template<...>` classes bound to the `*DynamicIDContainer` bases at the bottom of the header (include/ode/odecpp.h:1330).
- **`id()` and an implicit `operator dXxxID()` are the mixing bridge.** A wrapper passes straight into any C function that wants the raw handle, so you can freely interleave wrapper code and C code (include/ode/odecpp.h:70).
- **Ownership is RAII.** Each container's destructor frees its handle (`dWorldDestroy`/`dBodyDestroy`/`dJointDestroy`/`dJointGroupDestroy`/`dGeomDestroy`). Never also free a wrapped `id()` through the C API or you double-free (include/ode/odecpp_collision.h:45).
- **Wrappers are non-copyable** (copy ctor and `operator=` declared private/undefined), which is what prevents accidental double-free via value copies (include/ode/odecpp.h:59).
- **Some step methods discard the C status int.** `dWorld::step`/`quickStep` return `void`, throwing away the `int` success/failure that the underlying `dWorldStep`/`dWorldQuickStep` return — drop to the C API when you must check it (include/ode/odecpp.h:92).

## When to use

- Use the C++ wrappers when you want RAII cleanup of worlds/bodies/joints/geoms and terser construction (`dHingeJoint j(world); j.attach(a, b);`) instead of manual `dJointCreateHinge` + `dJointDestroy`.
- Mix wrappers with the C API freely: build a `dWorld`/`dHashSpace`/`dBody` as wrappers, then call any raw `dXxx*` function by passing the wrapper (implicit conversion) or `wrapper.id()` (include/ode/odecpp.h:70).
- Drop back to the **pure C API** when you need the `int` return of `dWorldStep`/`dWorldQuickStep`, when you need geoms/types absent here (`dMass` struct, trimesh, heightfield), or when you need manual lifetime control that conflicts with RAII.
- Do not use them in a C-only translation unit — the classes do not exist outside `__cplusplus` (include/ode/odecpp.h:28).

## Public API (C)

### World — `dWorld` (typedef of `dWorldTemplate<dWorldDynamicIDContainer>`)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dWorldTemplate::dWorldTemplate` | `dWorldTemplate()` | include/ode/odecpp.h:67 | Default ctor; immediately calls `dWorldCreate()` and stores the id. There is **no** `create()` method on the world. |
| `dWorldTemplate::id` | `dWorldID id() const` | include/ode/odecpp.h:70 | Returns the wrapped `dWorldID` for mixing with the C API. |
| `dWorldTemplate::operator dWorldID` | `operator dWorldID() const` | include/ode/odecpp.h:72 | Implicit conversion to the raw `dWorldID` handle. |
| `dWorldTemplate::setGravity` | `void setGravity(dReal x, dReal y, dReal z)` | include/ode/odecpp.h:75 | Wraps `dWorldSetGravity`; a `const dVector3` overload also exists. |
| `dWorldTemplate::step` | `void step(dReal stepsize)` | include/ode/odecpp.h:92 | Wraps `dWorldStep` (one big step). **Returns void — discards the C int status.** |
| `dWorldTemplate::quickStep` | `void quickStep(dReal stepsize)` | include/ode/odecpp.h:95 | Wraps `dWorldQuickStep` (iterative LCP solver). **Returns void — discards the C int status.** |
| `dWorldTemplate::impulseToForce` | `void impulseToForce(dReal stepsize, dReal ix, dReal iy, dReal iz, dVector3 force)` | include/ode/odecpp.h:156 | Wraps `dWorldImpulseToForce`. |

### Body — `dBody`

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dBodyTemplate::dBodyTemplate` | `dBodyTemplate(dWorldID world)` | include/ode/odecpp.h:197 | Ctor calls `dBodyCreate(world)`. A default no-arg ctor leaving `_id=0` also exists (declare-now, `create()`-later). |
| `dBodyTemplate::create` | `void create(dWorldID world)` | include/ode/odecpp.h:202 | Calls `destroy()` then `dBodyCreate` — safely re-creates an already-default-constructed body. |
| `dBodyTemplate::id` | `dBodyID id() const` | include/ode/odecpp.h:210 | Returns the wrapped `dBodyID`; `operator dBodyID()` is also provided. |
| `dBodyTemplate::setPosition` | `void setPosition(dReal x, dReal y, dReal z)` | include/ode/odecpp.h:220 | Wraps `dBodySetPosition`; a `dVector3` overload is provided. |
| `dBodyTemplate::setMass` | `void setMass(const dMass *mass)` | include/ode/odecpp.h:249 | Wraps `dBodySetMass`; a reference overload and `getMass()` returning `dMass` by value also exist. (`dMass` is used by value, not wrapped here.) |
| `dBodyTemplate::addForce` | `void addForce(dReal fx, dReal fy, dReal fz)` | include/ode/odecpp.h:256 | Wraps `dBodyAddForce`; many `addForceAtPos`/`addRelForce*` variants follow. |
| `dBodyTemplate::enable` | `void enable()` | include/ode/odecpp.h:318 | Wraps `dBodyEnable`; `disable()`/`isEnabled()` are also wrapped. |

### Joint group — `dJointGroup`

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointGroupTemplate::dJointGroupTemplate` | `dJointGroupTemplate()` | include/ode/odecpp.h:469 | Ctor calls `dJointGroupCreate(0)` (0 is the only valid argument). |
| `dJointGroupTemplate::empty` | `void empty()` | include/ode/odecpp.h:482 | Wraps `dJointGroupEmpty`; `clear()` is an alias. |

### Joint base — `dJoint` (abstract; cannot be instantiated)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointTemplate::attach` | `void attach(dBodyID body1, dBodyID body2)` | include/ode/odecpp.h:535 | Wraps `dJointAttach`; a wrapper-reference overload also exists. (There is no `detach()`; use `attach(0, 0)` semantics via the C API if needed.) |
| `dJointTemplate::getType` | `dJointType getType() const` | include/ode/odecpp.h:552 | Wraps `dJointGetType`. |
| `dJointTemplate::setParam` | `virtual void setParam(int, dReal)` | include/ode/odecpp.h:564 | Virtual base **no-op**; each joint subclass overrides it to call its `dJointSet<Type>Param`. (`getParam` is the matching virtual, returning 0 in the base.) |

### Joint subclasses (the instantiable joints)

All follow one pattern: a default no-arg ctor (`_id=0`), a `(dWorldID world, dJointGroupID group=0)` ctor, and a matching `create(world, group=0)`. `dContactJoint` is the exception (no group default, requires a `dContact*`).

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dBallJoint` | `typedef dBallJointTemplate<...>` | include/ode/odecpp.h:1334 | Ball-and-socket joint wrapper. |
| `dHingeJoint` | `typedef dHingeJointTemplate<...>` | include/ode/odecpp.h:1335 | Hinge joint wrapper. |
| `dSliderJoint` | `typedef dSliderJointTemplate<...>` | include/ode/odecpp.h:1336 | Slider joint wrapper. |
| `dUniversalJoint` | `typedef dUniversalJointTemplate<...>` | include/ode/odecpp.h:1337 | Universal joint wrapper. |
| `dHinge2Joint` | `typedef dHinge2JointTemplate<...>` | include/ode/odecpp.h:1338 | Hinge-2 joint wrapper. |
| `dPRJoint` | `typedef dPRJointTemplate<...>` | include/ode/odecpp.h:1339 | Prismatic-rotoide joint wrapper. |
| `dPUJoint` | `typedef dPUJointTemplate<...>` | include/ode/odecpp.h:1340 | Prismatic-universal joint wrapper. |
| `dPistonJoint` | `typedef dPistonJointTemplate<...>` | include/ode/odecpp.h:1341 | Piston joint wrapper. |
| `dFixedJoint` | `typedef dFixedJointTemplate<...>` | include/ode/odecpp.h:1342 | Fixed joint wrapper. |
| `dContactJoint` | `typedef dContactJointTemplate<...>` | include/ode/odecpp.h:1343 | Contact joint wrapper; ctor takes a `dContact*` (see below). |
| `dNullJoint` | `typedef dNullJointTemplate<...>` | include/ode/odecpp.h:1344 | Null joint wrapper. |
| `dAMotorJoint` | `typedef dAMotorJointTemplate<...>` | include/ode/odecpp.h:1345 | Angular motor joint wrapper. |
| `dLMotorJoint` | `typedef dLMotorJointTemplate<...>` | include/ode/odecpp.h:1346 | Linear motor joint wrapper. |
| `dBallJointTemplate::dBallJointTemplate` | `dBallJointTemplate(dWorldID world, dJointGroupID group=0)` | include/ode/odecpp.h:586 | Representative ctor: calls `dJointCreateBall(world, group)`; default no-arg ctor also exists. |
| `dBallJointTemplate::create` | `void create(dWorldID world, dJointGroupID group=0)` | include/ode/odecpp.h:591 | `destroy()` then `dJointCreateBall`; this create pattern is repeated for every joint subclass. |
| `dHingeJointTemplate::setAxis` | `void setAxis(dReal x, dReal y, dReal z)` | include/ode/odecpp.h:652 | Example subclass member: wraps `dJointSetHingeAxis`. |
| `dHinge2JointTemplate::setAxes` | `void setAxes(const dReal *axis1, const dReal *axis2)` | include/ode/odecpp.h:834 | Wraps `dJointSetHinge2Axes`; the older `setAxis1`/`setAxis2` are `ODE_API_DEPRECATED`. |
| `dContactJointTemplate::dContactJointTemplate` | `dContactJointTemplate(dWorldID world, dJointGroupID group, dContact *contact)` | include/ode/odecpp.h:1148 | Calls `dJointCreateContact`; group **and** contact are required (no defaults). |

### Geom base — `dGeom`

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dGeom::~dGeom` | `~dGeom()` | include/ode/odecpp_collision.h:45 | Destructor: if `_id` is set, calls `dGeomDestroy` — RAII frees the underlying geom. |
| `dGeom::id` | `dGeomID id() const` | include/ode/odecpp_collision.h:48 | Returns the wrapped `dGeomID`; `operator dGeomID()` also provided. |
| `dGeom::destroy` | `void destroy()` | include/ode/odecpp_collision.h:53 | Explicit `dGeomDestroy(_id)` then zero `_id` (the **only** public `destroy()` in this layer). |
| `dGeom::setBody` | `void setBody(dBodyID b)` | include/ode/odecpp_collision.h:69 | Wraps `dGeomSetBody`; `getBody()` returns `dBodyID`. |
| `dGeom::collide2` | `void collide2(dGeomID g, void *data, dNearCallback *callback)` | include/ode/odecpp_collision.h:131 | Wraps `dSpaceCollide2` (this geom vs another geom). |

### Space — `dSpace` (abstract) and concrete spaces

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dSpace` | abstract `dGeom` subclass (protected ctor) | include/ode/odecpp_collision.h:136 | Wraps `dSpaceID`; cannot be instanced directly. `id()`/`operator dSpaceID()` cast `_id` to `dSpaceID`. |
| `dSpace::setCleanup` | `void setCleanup(int mode)` | include/ode/odecpp_collision.h:153 | Wraps `dSpaceSetCleanup`; controls whether destroying the space destroys its geoms. |
| `dSpace::add` | `void add(dGeomID x)` | include/ode/odecpp_collision.h:158 | Wraps `dSpaceAdd`; `remove()`/`query()`/`getNumGeoms()`/`getGeom()` also wrapped. |
| `dSpace::collide` | `void collide(void *data, dNearCallback *callback)` | include/ode/odecpp_collision.h:170 | Wraps `dSpaceCollide` (broadphase over all geoms in this space). |
| `dSimpleSpace` | concrete `dSpace`; ctor calls `dSimpleSpaceCreate` | include/ode/odecpp_collision.h:175 | Brute-force space. |
| `dHashSpace` | concrete `dSpace`; ctor calls `dHashSpaceCreate`, adds `setLevels` | include/ode/odecpp_collision.h:190 | Spatial-hash space; `setLevels(min, max)`. |
| `dQuadTreeSpace` | `dQuadTreeSpace(const dVector3 center, const dVector3 extents, int depth)` | include/ode/odecpp_collision.h:208 | Quadtree space; ctor calls `dQuadTreeSpaceCreate(center, extents, depth)`. |

### Shape geoms (each is a `dGeom` subclass with ctor + `create()`)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dSphere` | `dGeom` subclass; ctor calls `dCreateSphere` | include/ode/odecpp_collision.h:223 | Sphere geom. |
| `dSphere::create` | `void create(dSpaceID space, dReal radius)` | include/ode/odecpp_collision.h:237 | Destroys old `_id` if present then `dCreateSphere`; same `create()` pattern on every geom. |
| `dBox` | `dGeom` subclass; ctor calls `dCreateBox` | include/ode/odecpp_collision.h:249 | Box geom. |
| `dPlane` | `dGeom` subclass; ctor calls `dCreatePlane(a,b,c,d)` | include/ode/odecpp_collision.h:275 | Infinite plane geom (non-placeable). |
| `dCapsule` | `dGeom` subclass; ctor calls `dCreateCapsule(radius,length)` | include/ode/odecpp_collision.h:301 | Capsule geom. |
| `dCylinder` | `dGeom` subclass; ctor calls `dCreateCylinder(radius,length)` | include/ode/odecpp_collision.h:327 | Cylinder geom. |
| `dRay` | `dGeom` subclass; ctor calls `dCreateRay(length)` | include/ode/odecpp_collision.h:353 | Ray geom. |
| `dRay::set` | `void set(dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)` | include/ode/odecpp_collision.h:377 | Wraps `dGeomRaySet` (origin + direction). |
| `dGeomTransform` | `ODE_API_DEPRECATED dGeom` subclass; ctor calls `dCreateGeomTransform` | include/ode/odecpp_collision.h:425 | **Deprecated** geom-transform wrapper. |

### ID-container base classes (you never name these directly)

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dWorldSimpleIDContainer` | base holding `dWorldID _id` | include/ode/odecpp.h:36 | Non-virtual dtor calls `destroy()`/`dWorldDestroy`. |
| `dWorldDynamicIDContainer` | adds a virtual destructor | include/ode/odecpp.h:51 | The default base used for the `dWorld` typedef. |
| `dBodySimpleIDContainer` | base holding `dBodyID _id` | include/ode/odecpp.h:162 | Dtor calls `dBodyDestroy`. |
| `dJointGroupSimpleIDContainer` | base holding `dJointGroupID _id` | include/ode/odecpp.h:436 | Dtor calls `dJointGroupDestroy`. |
| `dJointSimpleIDContainer` | base holding `dJointID _id` | include/ode/odecpp.h:489 | Dtor calls `dJointDestroy`. |

### The bridge / RAII / create patterns (verbatim)

```c
// id() / operator XxxID() bridge — pass a C++ wrapper straight into the C API
dWorldID id() const
  { return get_id(); }
operator dWorldID() const
  { return get_id(); }
```
(include/ode/odecpp.h:70)

```c
// dGeom RAII destructor (frees the underlying handle automatically)
~dGeom()
  { if (_id) dGeomDestroy (_id); }
...
void destroy() {
  if (_id) dGeomDestroy (_id);
  _id = 0;
}
```
(include/ode/odecpp_collision.h:45)

```c
// Joint subclass ctor + create() pattern (dBallJoint), group defaults to 0
dBallJointTemplate() { }
dBallJointTemplate (dWorldID world, dJointGroupID group=0)
  { set_id(dJointCreateBall(world, group)); }
void create (dWorldID world, dJointGroupID group=0) {
  destroy();
  set_id(dJointCreateBall(world, group));
}
```
(include/ode/odecpp.h:585)

```c
// Space-level broadphase collide() taking the dNearCallback
void collide (void *data, dNearCallback *callback)
  { dSpaceCollide (id(),data,callback); }
```
(include/ode/odecpp_collision.h:170)

## Key rules

- The entire C++ wrapper is guarded by `#ifdef __cplusplus` (include guards `_ODE_ODECPP_H_` / `_ODE_ODECPP_COLLISION_H_`); it does not exist in a C build (include/ode/odecpp.h:28).
- Usable class names (`dWorld`, `dBody`, `dJoint` subclasses, etc.) are `typedef`s of the `*Template` classes instantiated with the `*DynamicIDContainer` bases at the bottom of the header; write `dWorld`, not `dWorldTemplate` (include/ode/odecpp.h:1330).
- Every wrapper exposes `id()` and an implicit `operator XxxID()`, so a wrapper object passes directly to any C API function expecting the raw handle — this is the intended mixing bridge (include/ode/odecpp.h:70).
- `dWorld`'s destructor (via `dWorldSimpleIDContainer`) calls `dWorldDestroy` when `_id` is set — the world is RAII-owned and freed automatically (include/ode/odecpp.h:43).
- `dBody` destructor calls `dBodyDestroy`; `dJointGroup` destructor calls `dJointGroupDestroy`; `dJoint` destructor calls `dJointDestroy` — each container frees its own handle automatically (include/ode/odecpp.h:169).
- `dGeom` (and all geom/space subclasses) destructor calls `dGeomDestroy(_id)` when `_id` is set — geoms are RAII-owned; an explicit public `destroy()` is also available (include/ode/odecpp_collision.h:45).
- `dSpace`, `dWorld`, `dBody`, the `dJoint` base, and the `*IDContainer` types have private/undefined copy ctor and `operator=` (declared but intentionally undefined) — wrappers are non-copyable to prevent double-free (include/ode/odecpp.h:59).
- `dJointTemplate` has a protected default ctor and no public create-from-world ctor — you cannot instantiate a bare `dJoint`; construct a concrete subclass (`dBallJoint`, `dHingeJoint`, ...) (include/ode/odecpp.h:523).
- `dSpace` has a protected default ctor; you must instance a concrete subclass (`dSimpleSpace`/`dHashSpace`/`dQuadTreeSpace`), not `dSpace` itself (include/ode/odecpp_collision.h:142).
- Joint subclass `create()`/ctor takes a `dJointGroupID group` defaulting to `0` (no group); `dContactJoint` is the exception — its ctor/create require `group` AND a `dContact*` with no defaults (include/ode/odecpp.h:1148).
- `create()` on a joint/body calls `destroy()` first (freeing any existing handle); `create()` on a geom calls `dGeomDestroy` on the old `_id` first — so `create()` safely re-initializes an already-constructed wrapper (include/ode/odecpp.h:202).
- `setParam`/`getParam` are virtual on `dJoint`; the base versions are no-ops returning 0, and each joint subclass overrides them to call its `dJointSet<Type>Param`/`dJointGet<Type>Param` (include/ode/odecpp.h:564).
- `dGeomTransform` and `dRay::setParams`/`getParams(int*,int*)` are marked `ODE_API_DEPRECATED`; `dHinge2` `setAxis1`/`setAxis2` are also deprecated in favor of `setAxes` (include/ode/odecpp_collision.h:425).
- Default constructors that leave `_id=0` exist for `dBody` and all geom subclasses (`dSphere(){}`, `dBox(){}`, ...) so wrappers can be declared first and `create()`'d later (include/ode/odecpp_collision.h:229).
- `dWorld::step`/`quickStep` return `void`, silently discarding the `int` success/failure status the C functions report — use `dWorldStep(w.id(), h)` directly if you need to detect solver failure (include/ode/odecpp.h:92).

## Common mistakes

| Bad | Good | Why |
| --- | --- | --- |
| `dWorldTemplate<...> world; dHingeJointTemplate<...> j;` | `dWorld world; dHingeJoint j(world);` | The class you instantiate is the typedef; the `*Template` form requires supplying container template arguments and is never intended for user code (include/ode/odecpp.h:1330). |
| `dGeom g(...); dGeomDestroy(g.id()); /* then g's dtor runs */` | Let the wrapper's destructor (or `g.destroy()`) free it; never call `dGeomDestroy` on a wrapped id | Every wrapper has a RAII destructor that frees its handle; also destroying the same `id()` via the C API double-frees (include/ode/odecpp_collision.h:45). |
| `dBody make() { dBody b(world); return b; } // copy` | Construct in place; pass `dBody&` / `dGeomID` around | Wrappers are non-copyable (copy ctor and `operator=` private/undefined); copying fails to compile, and forcing it double-frees (include/ode/odecpp.h:59). |
| `dJoint j(world); dSpace space;` | `dBallJoint j(world); dHashSpace space;` | `dJoint` and `dSpace` have protected/no-public-create ctors and cannot be instantiated directly; use a concrete subclass (include/ode/odecpp.h:523). |
| `if (world.step(h) != 0) { ... }` | `if (dWorldStep(world.id(), h) != 0) { ... }` | `dWorld::step`/`quickStep` return `void` and discard the C `int` status; only the raw C call exposes success/failure (include/ode/odecpp.h:92). |

## Named constants

These are preprocessor macros that select the container base for each template; the defaults are auto-applied, and either all four must be defined together or none (the header `#error`s otherwise).

| Name | Value | Source | Purpose |
| --- | --- | --- | --- |
| `dODECPP_WORLD_TEMPLATE_BASE` | `dWorldDynamicIDContainer` (default) | include/ode/odecpp.h:1316 | Selects the world container base; all four template-base macros must be defined together or none. |
| `dODECPP_BODY_TEMPLATE_BASE` | `dBodyDynamicIDContainer` (default) | include/ode/odecpp.h:1317 | Selects the body container base. |
| `dODECPP_JOINTGROUP_TEMPLATE_BASE` | `dJointGroupDynamicIDContainer` (default) | include/ode/odecpp.h:1318 | Selects the joint-group container base. |
| `dODECPP_JOINT_TEMPLATE_BASE` | `dJointDynamicIDContainer` (default) | include/ode/odecpp.h:1319 | Selects the joint container base. |

## Things to never invent

- `dWorld::create()` — `dWorld`/`dWorldTemplate` has **no** `create()` method (only a default ctor that calls `dWorldCreate`); only `dBody`, `dJointGroup`, the joint subclasses, and the geom subclasses have `create()`.
- `dBody::destroy()` as a public method — `destroy()` is **protected** on the dynamics wrappers (only `dGeom` exposes a public `destroy()`).
- `dJoint::detach()` — no such method; only `attach()` exists.
- `dWorld::setStepSize()` / `dWorld::getStepSize()` — no such wrappers; stepsize is an argument to `step()`/`quickStep()`.
- `dSpace::destroy()` as a distinct method — spaces are destroyed via the inherited `dGeom` destructor/`destroy()`.
- `dContactJoint` default ctor with a `(world, group)` signature — the contact joint requires a `dContact*` and has no group-only ctor.
- `dTriMesh` / `dHeightfield` wrapper classes — not present in odecpp_collision.h.
- `dMass` as a wrapper class in these headers — `dMass` is used by value (`dBody::setMass`/`getMass`) but is not defined here (it lives in include/ode/mass.h).
- `dWorld::collide()` — collision lives on `dSpace`/`dGeom`, not `dWorld`.

## Cross-header notes (verified, not in these two files)

- `dMass` is referenced (`dBodyTemplate::setMass`/`getMass` take/return `dMass`) but its struct definition is **not** in these two headers — it is declared in include/ode/mass.h, outside this module's scope.
- `ODE_API_DEPRECATED`, `dReal`, `dVector3`, `dVector4`, `dMatrix3`, `dQuaternion`, `dContact`, `dJointFeedback`, `dNearCallback`, `dJointType`, and all `dXxxID` handle typedefs are used by the wrappers but defined in other headers (common.h, contact.h, collision.h).
- The underlying C functions (`dWorldCreate`, `dBodyCreate`, `dJointCreateBall`, `dCreateSphere`, `dSpaceCollide`, etc.) are only *called* by these inline wrappers; their canonical declarations live in objects.h / collision.h / collision_space.h.

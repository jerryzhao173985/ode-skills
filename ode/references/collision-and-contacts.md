# Collision and Contacts

> Source of truth: include/ode/collision.h, include/ode/collision_space.h, include/ode/contact.h, include/ode/objects.h (joint group + contact-joint API). Every rule cites real ODE source by file:line; headers win over the wiki on conflict. C symbols start with 'd'; do not invent symbols.

## Mental model

- Collision detection is **decoupled** from dynamics. The collision side produces `dContactGeom` points; you turn each into a **contact joint** that the world step then solves. Nothing happens automatically — you write the bridge in a callback.
- `dSpaceCollide(space, data, &nearCallback)` does broadphase only: it hands your callback **candidate** pairs (`o1`, `o2`) that *might* touch. You call `dCollide(o1,o2,...)` to get the actual contact points.
- `dCollide` fills an array of `dContactGeom` and **returns the count** (0 = no intersection). Always check `n > 0` — candidate pairs are not guaranteed to collide.
- `dContact.surface.mode` is a **bitmask**: `mode` and `mu` are always read; every other surface field (`bounce`, `soft_cfm`, `slip1`, `mu2`, …) is read **only if** its mode bit is set. Set the bit *and* the field.
- Contact joints are **single-step**: created in a per-step joint group (`dJointCreateContact`), attached (`dJointAttach`), then the **entire group is emptied** (`dJointGroupEmpty`) after the world step. The group object persists; its joints do not.
- Geom creation, placement, spaces (`dCreateBox`, `dHashSpaceCreate`, `dGeomSetBody`, category/collide bits, heightfields, geom transforms) live in **references/geoms-and-spaces.md** — this file covers the collide → contact → joint pipeline only.

## When to use

- You need bodies to actually push on each other / rest on the ground: wire `dSpaceCollide` → `nearCallback` → `dCollide` → `dJointCreateContact` → `dJointAttach`, then `dWorldStep`/`dWorldQuickStep`, then `dJointGroupEmpty`.
- You want to tune contact behavior — friction (`mu`/`mu2`), bounce, soft constraints (ERP/CFM), slip, surface motion — by setting `surface.mode` bits and the matching fields.
- You want fine control over which pairs collide (filter inside the callback, or via category/collide bits — see geoms-and-spaces.md).
- You need raw geom-vs-geom intersection without joints (e.g. ray queries, trigger volumes): call `dCollide` directly and read the `dContactGeom` array.

## Public API (C)

### Broadphase and narrowphase

| Symbol | Signature | Source | Purpose |
|--------|-----------|--------|---------|
| `dCollide` | `int dCollide (dGeomID o1, dGeomID o2, int flags, dContactGeom *contact, int skip)` | include/ode/collision.h:792 | Generate up to `(flags & 0xffff)` contact points between two geoms; returns the number generated (0 if no intersection / same geom). `contact` is a `dContactGeom` array; `skip` is the byte stride between entries (>= `sizeof(dContactGeom)`). |
| `dSpaceCollide` | `void dSpaceCollide (dSpaceID space, void *data, dNearCallback *callback)` | include/ode/collision.h:822 | Broadphase one space: invoke `callback(data,o1,o2)` for each candidate pair. Does NOT recurse into nested sub-spaces. May report close-but-non-intersecting pairs. |
| `dSpaceCollide2` | `void dSpaceCollide2 (dGeomID space1, dGeomID space2, void *data, dNearCallback *callback)` | include/ode/collision.h:865 | Cross-collide geoms of two spaces (or a geom vs a space); invoke callback per candidate pair. Use it to descend into a sub-space that `dSpaceCollide` handed you. |

### Contact joints and the per-step joint group (declared in objects.h)

| Symbol | Signature | Source | Purpose |
|--------|-----------|--------|---------|
| `dJointCreateContact` | `dJointID dJointCreateContact (dWorldID, dJointGroupID, const dContact *)` | include/ode/objects.h:1717 | Create one contact joint from a filled `dContact`, allocated in the given joint group. One per contact returned by `dCollide`. |
| `dJointGroupCreate` | `dJointGroupID dJointGroupCreate (int max_size)` | include/ode/objects.h:1835 | Create a joint group that holds per-step contact joints; persists across steps. Header: `max_size` is deprecated — set to 0. |
| `dJointGroupEmpty` | `void dJointGroupEmpty (dJointGroupID)` | include/ode/objects.h:1852 | Destroy all joints in the group **without** destroying the group itself. Call once per step after the world step. |
| `dJointGroupDestroy` | `void dJointGroupDestroy (dJointGroupID)` | include/ode/objects.h:1843 | Destroy the joint group and all its joints (shutdown cleanup). |
| `dJointAttach` | `void dJointAttach (dJointID, dBodyID body1, dBodyID body2)` | (declared in include/ode/objects.h; see joints reference) | Attach a (contact) joint to two bodies. Pass `dGeomGetBody(geom.g1)` / `g2`; a static geom returns body 0 = the static environment. [VERIFY: exact objects.h line not in this inventory] |

> `dGeomGetBody`, `dGeomSetBody`, and all `dCreate*` geom/space constructors are documented in **references/geoms-and-spaces.md**.

## Key data structures (C)

| Type | Layout | Source | Purpose |
|------|--------|--------|---------|
| `dContactGeom` | `dVector3 pos; dVector3 normal; dReal depth; dGeomID g1,g2; int side1,side2;` | include/ode/contact.h:88 | Written by `dCollide` per contact point. `normal` points **into** `g1`; moving `g1` along `normal` by `depth` brings penetration to zero. |
| `dContact` | `dSurfaceParameters surface; dContactGeom geom; dVector3 fdir1;` | include/ode/contact.h:99 | Full contact you pass to `dJointCreateContact`. `dCollide` fills the embedded `.geom`; you fill `.surface`; `.fdir1` is the first friction direction (only if `dContactFDir1`). |
| `dSurfaceParameters` | `int mode; dReal mu; dReal mu2, rho, rho2, rhoN, bounce, bounce_vel, soft_erp, soft_cfm, motion1,motion2,motionN, slip1,slip2;` | include/ode/contact.h:55 | Material parameters. `mode` and `mu` are **always** read; every field after `mu` is read only if its mode bit is set. |
| `dNearCallback` | `typedef void dNearCallback (void *data, dGeomID o1, dGeomID o2)` | include/ode/collision_space.h:49 | User callback invoked by `dSpaceCollide`/`dSpaceCollide2` per candidate pair; you call `dCollide` inside it. |

## surface.mode bitflags (every flag in contact.h)

| Flag | Value | Source | Gates which field(s) | Meaning |
|------|-------|--------|----------------------|---------|
| `dContactMu2` | `0x001` | include/ode/contact.h:34 | `surface.mu2` | Axis-dependent friction (second friction direction). |
| `dContactAxisDep` | `0x001` | include/ode/contact.h:35 | `surface.mu2` | Alias of `dContactMu2` (same bit). |
| `dContactFDir1` | `0x002` | include/ode/contact.h:36 | `dContact.fdir1` | Use `fdir1` as the first friction direction. |
| `dContactBounce` | `0x004` | include/ode/contact.h:37 | `surface.bounce`, `surface.bounce_vel` | Restitution: restore collision energy anti-parallel to the normal. |
| `dContactSoftERP` | `0x008` | include/ode/contact.h:38 | `surface.soft_erp` | Use `soft_erp` instead of the global ERP for this contact. |
| `dContactSoftCFM` | `0x010` | include/ode/contact.h:39 | `surface.soft_cfm` | Use `soft_cfm` instead of the global CFM for this contact. |
| `dContactMotion1` | `0x020` | include/ode/contact.h:40 | `surface.motion1` | Non-zero target surface velocity, friction direction 1. |
| `dContactMotion2` | `0x040` | include/ode/contact.h:41 | `surface.motion2` | Target surface velocity, friction direction 2. |
| `dContactMotionN` | `0x080` | include/ode/contact.h:42 | `surface.motionN` | Target surface velocity along the normal. |
| `dContactSlip1` | `0x100` | include/ode/contact.h:43 | `surface.slip1` | Force-dependent slip, friction direction 1. |
| `dContactSlip2` | `0x200` | include/ode/contact.h:44 | `surface.slip2` | Force-dependent slip, friction direction 2. |
| `dContactRolling` | `0x400` | include/ode/contact.h:45 | `surface.rho`, `rho2`, `rhoN` | Rolling / angular friction. |
| `dContactApprox0` | `0x0000` | include/ode/contact.h:47 | — | No friction-cone approximation (default; the friction box). |
| `dContactApprox1_1` | `0x1000` | include/ode/contact.h:48 | friction dir 1 | Pyramid approximation for friction direction 1. |
| `dContactApprox1_2` | `0x2000` | include/ode/contact.h:49 | friction dir 2 | Pyramid approximation for friction direction 2. |
| `dContactApprox1_N` | `0x4000` | include/ode/contact.h:50 | normal/rolling | Pyramid approximation for rolling friction. |
| `dContactApprox1` | `0x7000` | include/ode/contact.h:51 | dirs 1, 2, N | All three pyramid-approximation bits combined. [VERIFY: "friction pyramid vs box" semantics are wiki-derived; the header only labels the 0x7000 bits.] |

## Key rules

- Per-step pipeline order: build/update geoms → `dSpaceCollide(space, data, &nearCallback)` (callback runs here, **before** the world step) → inside the callback `dCollide()` then `dJointCreateContact()` + `dJointAttach()` → `dWorldStep`/`dWorldQuickStep` → `dJointGroupEmpty(contactgroup)` (ode/demo/demo_boxstack.cpp:508).
- `dCollide` writes into a `dContactGeom` array; the low 16 bits of `flags` set the max contact count (you must ask for at least one), and `skip` is the byte stride between entries (must be >= `sizeof(dContactGeom)`). Passing `&contact[0].geom` with `skip = sizeof(dContact)` fills the embedded `geom` of each `dContact` (include/ode/collision.h:792).
- `dCollide` returns the number of contacts generated (0 if no intersection, same geom, or self); only iterate `[0, n)` — the rest of the array is untouched (include/ode/collision.h:773).
- For each contact, create a contact joint with `dJointCreateContact` in a per-step joint group, then `dJointAttach` it to `dGeomGetBody(geom.g1)` and `dGeomGetBody(geom.g2)`; a static geom (e.g. a plane) returns body 0, which `dJointAttach` treats as the static environment (ode/demo/demo_buggy.cpp:105).
- You MUST call `dJointGroupEmpty(contactgroup)` once per step **after** the world step to destroy that step's contact joints; the group itself is created once via `dJointGroupCreate(0)` and destroyed at shutdown with `dJointGroupDestroy` (ode/demo/demo_boxstack.cpp:548).
- `surface.mode` is a bitmask gating which surface fields are read: `mode` and `mu` are ALWAYS read; `mu2` only with `dContactMu2`, `bounce`/`bounce_vel` only with `dContactBounce`, `soft_cfm` only with `dContactSoftCFM`, `slip1`/`slip2` only with `dContactSlip1`/`dContactSlip2`, etc. Set the field only after setting its mode bit (include/ode/contact.h:60).
- Spaces are themselves geoms; `dSpaceCollide` does NOT recurse into nested sub-spaces — they may be passed to the callback as `o1`/`o2`, and you must call `dSpaceCollide2` (or `dCollide`) on them yourself to descend (include/ode/collision.h:807).
- `dSpaceCollide`/`dSpaceCollide2` may report close-but-non-intersecting pairs, so `dCollide` can return 0 for a pair the callback received — always check `n > 0` before building joints (include/ode/collision.h:812).
- `CONTACTS_UNIMPORTANT` (`0x80000000`) may be OR'd into the `dCollide` flags to skip contact refining and just generate any contacts; all bits other than the low 16 (count) and this must be zero (include/ode/collision.h:743).
- Filter unwanted pairs inside the callback: e.g. skip pairs whose bodies are already connected by a joint via `if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;` (ode/demo/demo_boxstack.cpp:140).

## Pattern: per-material contacts (how real robot simulators do it)

To give each object its own friction/bounce/softness (wood vs ice; a snake's anisotropic belly), attach a
**material/owner to each geom** and recover it in the callback instead of hard-coding surface params:
- **At creation:** `dGeomSetData(geom, owner)` stores a `void*` on the geom (e.g. your `Primitive*` or a
  `Material*`).
- **In the near-callback:** `dGeomGetData(o1)` / `dGeomGetData(o2)` recover both owners, **combine** their
  materials (e.g. `mu = min(mu1, mu2)`; hardness via the spring-damper → ERP/CFM law), and write the result
  into each `contact[i].surface` *before* `dJointCreateContact`. Surface params are then computed **per
  colliding pair**, not fixed — the linchpin of a material-based contact model (e.g. lpzrobots' `Substance`).
  `dGeomSetData`/`dGeomGetData` are in `include/ode/collision.h`; the hardness-combination law is in
  `references/foundations/erp-cfm-friction.md`.
- **Direction-dependent friction** (a snake / tracked robot): set `dContactFDir1` in `surface.mode` and
  `contact.fdir1` along the low-friction axis, with `mu` (along `fdir1`) ≠ `mu2` (perpendicular).

## Canonical pipeline code

**nearCallback + dCollide + contact-joint build (demo_buggy)** — ground-only filter, slip + soft contacts (ode/demo/demo_buggy.cpp:84):

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}
```

**simLoop step sequence — dSpaceCollide / world step / dJointGroupEmpty (demo_buggy; this demo uses `dWorldStep`)** (ode/demo/demo_buggy.cpp:185):

```c
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
```

**nearCallback guarding against already-jointed bodies + per-contact bounce/soft surface (demo_boxstack)** (ode/demo/demo_boxstack.cpp:131):

```c
static void nearCallback (void *, dGeomID o1, dGeomID o2)
{
    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
        return;

    dContact contact[MAX_CONTACTS];
    for (i=0; i<MAX_CONTACTS; i++) {
        contact[i].surface.mode = dContactBounce | dContactSoftCFM;
        contact[i].surface.mu = dInfinity;
        contact[i].surface.mu2 = 0;
        contact[i].surface.bounce = 0.1;
        contact[i].surface.bounce_vel = 0.1;
        contact[i].surface.soft_cfm = 0.01;
    }
    if (int numc = dCollide(o1,o2,MAX_CONTACTS,&contact[0].geom,
                            sizeof(dContact))) {
        for (i=0; i<numc; i++) {
            dJointID c = dJointCreateContact (world,contactgroup,contact+i);
            dJointAttach (c,b1,b2);
        }
    }
}
```

**dWorldQuickStep variant of the same per-step sequence (demo_boxstack)** (ode/demo/demo_boxstack.cpp:508):

```c
static void simLoop(int pause)
{
    dSpaceCollide(space, 0, &nearCallback);

    if (!pause)
        dWorldQuickStep(world, 0.02);

    // ... (feedback handling) ...

    // remove all contact joints
    dJointGroupEmpty(contactgroup);
}
```

## Common mistakes

| Bad | Good | Why |
|-----|------|-----|
| `dSpaceCollide(space,0,&nearCallback); dWorldQuickStep(world,0.02); /* (no empty) */` | `dSpaceCollide(space,0,&nearCallback); dWorldQuickStep(world,0.02); dJointGroupEmpty(contactgroup);` | Forgetting `dJointGroupEmpty` after the world step leaks contact joints every frame; they accumulate and the simulation blows up. It must run once per step after the step, and the group is created once outside the loop (ode/demo/demo_boxstack.cpp:548). |
| `contact.surface.bounce = 0.5; // mode left as 0` | `contact.surface.mode = dContactBounce; contact.surface.bounce = 0.5; contact.surface.bounce_vel = 0.1;` | Setting a surface field without setting its mode bit means it is silently ignored — `mu` is always read, but `bounce`/`soft_cfm`/`slip`/`mu2` are only honored when their bit is in `surface.mode` (include/ode/contact.h:60). |
| `dCollide(o1,o2,N,contact,sizeof(dContactGeom)); // contact is dContact[]` | `dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));` | The array is `dContactGeom`; to fill an array of `dContact` you pass the address of the first `.geom` with the `dContact` stride. Wrong stride writes over neighboring `surface`/`fdir1` data (include/ode/collision.h:792). |
| `n = dCollide(...); for(i=0;i<n;i++){ dJointCreateContact(...); } // no n check` | `n = dCollide(...); if (n > 0) { for(i=0;i<n;i++){ dJointCreateContact(...); } }` | `dSpaceCollide` reports CANDIDATE pairs only; `dCollide` may return 0, so a guaranteed-contact assumption builds joints from stale/untouched array entries (include/ode/collision.h:812). |
| `dContactGroupEmpty(contactgroup);` | `dJointGroupEmpty(contactgroup);` | There is no `dContactGroupEmpty`; contact joints live in a *joint* group, emptied with `dJointGroupEmpty` (include/ode/objects.h:1852). |

## Named constants

> The full `surface.mode` bitflag set (`dContactMu2` … `dContactApprox1`, contact.h:34–51) is in the canonical **surface.mode bitflags** table above; not repeated here.

| Name | Value | Source | Purpose |
|------|-------|--------|---------|
| `CONTACTS_UNIMPORTANT` | `0x80000000` | include/ode/collision.h:743 | OR into `dCollide` flags to skip contact refining. |
| `dJointTypeContact` | (joint-type enum) | include/ode/common.h:389 | Joint type used with `dAreConnectedExcluding` to skip already-contacting pairs. |

> Geom-class ids (`dSphereClass`, `dBoxClass`, …) and space-class ids returned by `dGeomGetClass`, plus the `dSAP_AXES_*` macros, are documented in **references/geoms-and-spaces.md** (include/ode/collision.h:877, include/ode/collision_space.h:60).

## Things to never invent

Contact-pipeline-specific (geom/space invented-name traps — `dGeomCollide`, `dSpaceCollideAll`/`dWorldCollide`, `dSpaceStep`/`dSpaceUpdate`, `dGeomSetCollisionBits`, `dCreateCCylinder` — live in **references/geoms-and-spaces.md**):

- `dContactSoft` — no such flag. It is `dContactSoftERP` and/or `dContactSoftCFM`.
- `dContactFriction` / `dContactFriction2` — do not exist; use `dContactMu2` with `surface.mu2`.
- `dContactGroupEmpty` — the correct name is `dJointGroupEmpty`.
- `dContactGroupCreate` — the per-step container is a joint group: `dJointGroupCreate` / `dJointGroupEmpty` / `dJointGroupDestroy`.
- Treating `dContactApprox1` as a documented "friction pyramid vs box" toggle from the header — the header (include/ode/contact.h:51) only labels the `0x7000` bits; the pyramid-vs-box semantics are wiki-derived.

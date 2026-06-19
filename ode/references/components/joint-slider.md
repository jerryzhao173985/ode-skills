# Joint: Slider (prismatic)  (`dJointCreateSlider`)

> Source of truth: include/ode/objects.h:1709. Cited by file:line; do not invent symbols.

## What it is

A slider (prismatic) joint allows **1 translational DOF**: the two bodies may slide relative to each other along a single shared axis but cannot rotate relative to one another. It removes 5 DOF (3 rotational + 2 translational), leaving translation along the slider axis free. A slider has **only an axis — no anchor** (the position is a 1-D extension along that axis).

## Create & attach

```c
dJointID slider = dJointCreateSlider (world, 0);
dJointAttach (slider, body1, body2);               /* REQUIRED; 0 for a body => static env */
dJointSetSliderAxis (slider, ax, ay, az);          /* slide axis in WORLD coords */
```

Verbatim shape from `ode/demo/demo_slider.cpp:161-163`:

```c
slider = dJointCreateSlider (world,0);
dJointAttach (slider,body[0],body[1]);
dJointSetSliderAxis (slider,1,1,1);
```

## Type-specific API

| Symbol | Signature | Source | Purpose |
| --- | --- | --- | --- |
| `dJointSetSliderAxis` | `void dJointSetSliderAxis (dJointID, dReal x, dReal y, dReal z)` | include/ode/objects.h:2043 | Set the slider's translation axis in world coordinates. |
| `dJointSetSliderParam` | `void dJointSetSliderParam (dJointID, int parameter, dReal value)` | include/ode/objects.h:2054 | Set a slider limit/motor parameter via a `dParam*` constant. |
| `dJointGetSliderPosition` | `dReal dJointGetSliderPosition (dJointID)` | include/ode/objects.h:2637 | Get the linear extension relative to the zero set when the axis was assigned. |
| `dJointGetSliderAxis` | `void dJointGetSliderAxis (dJointID, dVector3 result)` | include/ode/objects.h:2649 | Read the current slider axis (writes into `result`). |

## Parameters / conventions

- **Axis only**: the slider has no anchor. The reported position is the distance along the slider axis of body1 with respect to body2 (a NULL body is replaced by the world); when the axis is set, the current pose becomes the **zero position** (`include/ode/objects.h:2637` doc block).
- **Stops** (apply via `dJointSetSliderParam`): `dParamLoStop` / `dParamHiStop` are **linear limits** here (include/ode/common.h:441-442), with `dParamBounce` / `dParamStopERP` / `dParamStopCFM` for stop behavior (include/ode/common.h:448-451).
- **Motor**: `dParamVel` is a **linear** desired velocity (include/ode/common.h:443) with `dParamFMax` the max force (include/ode/common.h:446). No force unless `dParamFMax > 0`.

## Pitfalls

- Stops on a slider are **distances**, not angles — `dParamLoStop`/`dParamHiStop` set the travel limits along the axis (`include/ode/common.h:441-442`).
- Setting `dParamVel` without `dParamFMax > 0` produces no motion (motor mistake; inventory mistakes block).
- Forgetting `dJointAttach` leaves the joint inert (`include/ode/objects.h:1688`); 0 for a body anchors to the static environment (`include/ode/objects.h:1866`).

## Never invent

- `dJointSetSliderAnchor` / `dJointGetSliderAnchor` — a slider has no anchor, only an axis.
- `dJointSetSliderMotor` / `dParamMaxForce` — use `dJointSetSliderParam` with `dParamVel` / `dParamFMax`.
- `dJointGetSliderAngle` — a slider measures linear position (`dJointGetSliderPosition`), not an angle.

See also the joints overview: `references/joints.md`.

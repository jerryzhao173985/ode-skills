# Example: feedback — measuring constraint forces & breaking overloaded joints

> Source: `ode/demo/demo_feedback.cpp` (313 lines). Verified line anchors. The demo for **`dJointFeedback`** and force-driven joint breaking.

## What it builds
A bridge of box segments chained by hinge joints (ends anchored by sliders), loaded by a falling stack. Each hinge reports its reaction force via feedback; a hinge that stays overloaded for 4 consecutive sub-steps is snapped.

## Components composed
- **World** + **Space** + contact **JointGroup**; **Body** segments (box **Geom** + **Mass**).
- **Joint**: `dJointCreateHinge` chain + `dJointCreateSlider` end anchors.
- **Feedback**: a caller-owned `dJointFeedback[]` array registered with `dJointSetFeedback`.

## Key calls (walkthrough)
| Line | Call | Role |
|---|---|---|
| 61 | `static dJointFeedback jfeedbacks[...]` | **caller-owned** buffer (must outlive the joints) |
| 251–255 | `dJointCreateHinge` + anchor/axis + `dParamFMax=8000` | bridge hinge |
| 258 | `dJointSetFeedback(hinges[i], jfeedbacks+i)` | register the slot |
| 235 | `dWorldSetQuickStepNumIterations(world,20)` | meaningful forces in a stiff scene |
| 185–187 | `dSpaceCollide` → `dWorldQuickStep(world,simstep)` → `dJointGroupEmpty` | the loop (2ms sub-steps) |
| 154–155 | `dCalcVectorLength3(jfeedbacks[i].f1)` | read force magnitude **after** the step |
| 151 / 167 | `dJointGetBody(h,0)` / `dJointAttach(h,0,0)` | check attached / **break** the joint |

## Patterns to copy
- **Joint feedback:** declare a persistent `dJointFeedback` array, `dJointSetFeedback(joint, &arr[i])`, then read `f1/t1/f2/t2` **after** `dWorldStep`/`QuickStep` populates them.
- **Break a joint:** `dJointAttach(joint, 0, 0)` detaches it from both bodies.
- **Fixed sub-stepping:** split the frame into fixed 2ms steps (lines 180–189) so physics is framerate-independent.
- **Low-pass the force** and require N consecutive over-threshold steps before reacting (forces are noisy).

## Gotchas
- The `dJointFeedback` array is **yours**, not ODE's — `dJointSetFeedback` only stores the pointer; it must stay valid for the whole sim.
- Read feedback **after** the step, inside the sub-step loop (not once per render frame).
- The near-callback recurses into sub-spaces with `dSpaceCollide2`; teardown order is geoms → space → world.

## See also
`references/joints.md` (`dJointFeedback`, `dJointSetFeedback`), `references/components/{joint,contact}.md`, `references/math-and-rotation.md` (`dCalcVectorLength3`).

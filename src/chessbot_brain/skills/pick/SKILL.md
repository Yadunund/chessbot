---
name: pick
description: Grasp the piece at a point on the table and lift it to transit height.
kind: manipulation
inputs:
  xyz: "grasp point on the playing surface, metres, robot base frame"
outputs: none
preconditions:
  - gripper is empty
  - calibration is available (the point comes from board geometry)
  - the point is reachable with a near-vertical tool
postconditions:
  - gripper closed on the piece
  - tool at transit height above the point
errors:
  - "CapabilityError: no IK solution for (...)  - point unreachable"
  - "CapabilityError: Cartesian path only N% feasible  - descent or lift blocked"
  - "CapabilityError: trajectory failed / arm controller is not available"
uses: [gripper, solve_ik, joint_move_trajectory, execute, plan_cartesian]
---

# pick

1. Open the gripper.
2. Joint-space move to hover above the point, as high as is reachable: 8 cm, else 6.5 cm, else 5 cm. Close to the robot's base the higher hover is out of reach.
3. Straight line down to `GRASP_HEIGHT` (1.5 cm).
4. Close the gripper. A stall on the piece counts as done.
5. Straight line back up to the same hover height.

```python
from chessbot_brain import skills
skills.pick(caps, geometry.square_centre("e7"))
```

**Limits:**
- Grasp success isn't verified yet: the gripper stalling is taken as success.
- The approach is vertical where possible; at the edge of reach the motion capability tilts the tool up to its `max_approach_tilt_deg`.

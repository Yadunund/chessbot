---
name: place
description: Lower the held piece onto a point and release it, ending at transit height.
kind: manipulation
inputs:
  xyz: "placement point on the playing surface, metres, robot base frame"
outputs: none
preconditions:
  - a piece is held (after pick)
  - the point is free and reachable
postconditions:
  - gripper open, piece released at the point
  - tool at transit height above the point
errors:
  - "CapabilityError: no IK solution / Cartesian path infeasible / trajectory failed"
uses: [solve_ik, joint_move_trajectory, execute, plan_cartesian, gripper]
---

# place

1. Joint-space move to hover above the point.
2. Straight line down to grasp height.
3. Open the gripper.
4. Straight line back up to transit height.

```python
skills.place(caps, geometry.square_centre("e5"))
```

**Limits:** placement accuracy isn't verified yet. Verification belongs to the caller, via perception.

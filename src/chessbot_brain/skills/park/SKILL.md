---
name: park
description: Move the arm to its parked joint configuration, clear of the board and the overhead camera's view.
kind: motion
inputs:
  park_joints: "joint positions, in the order of the arm joints"
outputs: none
preconditions: [arm controller available, joint states received]
postconditions: [arm at the park configuration]
errors: ["CapabilityError: trajectory failed / arm controller is not available"]
uses: [joint_move_trajectory, execute]
---

# park

Joint-space move, timed so no joint exceeds the configured speed.

```python
skills.park(caps, brain.park_joints)
```

---
name: transfer
description: Move one piece from one point to another (pick then place).
kind: manipulation
inputs:
  src_xyz: "where the piece is"
  dst_xyz: "where it goes"
outputs: none
preconditions: [src holds a piece, dst is free, both reachable]
postconditions: [piece at dst, gripper open, tool at transit height above dst]
errors: [any error from pick or place]
uses: [pick, place]
---

# transfer

```python
skills.transfer(caps, geometry.square_centre("e7"), geometry.square_centre("e5"))
```

Graveyard slots are points too: `geometry.graveyard_slot("white", n)`.

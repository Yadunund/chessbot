---
name: execute_chess_move
description: Carry out a chess move on the physical board, including its side effects (capture, castling rook, en passant).
kind: game
inputs:
  effects: "MoveEffects from Board.effects(uci): the move plus what it captures and moves"
  geometry: "BoardGeometry built from calibration"
  graveyard_counts: "next free graveyard slot per colour; updated in place"
  narrate: "callback receiving one human-readable line per step (the thought feed)"
outputs: none
preconditions:
  - the physical board matches the position the move is legal in
  - calibration is available
postconditions:
  - pieces physically moved; captured piece in the next graveyard slot
errors: [any error from transfer]
uses: [transfer]
---

# execute_chess_move

Order of operations:
1. **Capture:** the captured piece goes to the next free graveyard slot for its colour. For en passant it is taken from the square beside the destination.
2. **Main move:** the moving piece goes from source to destination.
3. **Castling:** the rook moves second.
4. **Promotion:** not handled physically yet. The thought feed asks for the piece to be swapped by hand.

```python
effects = board.effects("e7e5")
skills.execute_chess_move(caps, geometry, effects, graveyard, narrate=print)
board.apply("e7e5")  # update the logical board only after the physical move succeeded
```

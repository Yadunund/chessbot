# Skills

A skill is a sequence composed from capabilities. Skills never wait on a human
and keep no state between calls: they do one physical thing, then return or raise
`CapabilityError`. The brain's game logic decides what a failure means.

Code lives in `chessbot_brain/skills.py`; each directory here documents one
skill's contract in a `SKILL.md`, in the Agent Skills format (a `name` and
`description` in the frontmatter) so a reasoning agent can load them.

| Skill | What it does |
|---|---|
| [pick](pick/SKILL.md) | Grasp the piece at a point and lift it to transit height |
| [place](place/SKILL.md) | Lower the held piece onto a point and release it |
| [transfer](transfer/SKILL.md) | Pick at one point, place at another |
| [park](park/SKILL.md) | Move the arm to its parked configuration, out of the camera's view |
| [execute_chess_move](execute_chess_move/SKILL.md) | Carry out a chess move physically, including captures and castling |

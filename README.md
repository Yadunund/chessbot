<h1 align="center">chessbot</h1>

<p align="center">
  <b>Play chess against a real robot arm, on a real board.</b><br/>
  Move your piece, press the clock, and watch the robot think and reply.
</p>

<p align="center">
  <img src="https://github.com/Yadunund/chessbot/raw/media/ui-3d-moving.png" alt="The chessbot UI: a 3D view of the board with the SO-101 arm reaching for a piece while playing black" width="900"/>
</p>

## What it does

- **A robot opponent.** An [SO-101](https://github.com/TheRobotStudio/SO-ARM100) arm picks and places
  its own pieces, captures included, on a standard 21 cm board. Stockfish chooses its moves at the
  strength you pick and checks yours are legal.
- **See what the robot believes.** The UI draws the board, the pieces and the arm in 3D, with the arm
  moving live as the robot moves. Switch to top down or a flat 2D board at any time.
- **A clock you actually press.** When you've moved, hit the big clock button: the robot reads your move
  with its overhead camera. Player strips show who plays which colour, whose turn it is, and what the
  robot is doing.
- **Hear it think.** A running thought feed narrates each step: reading the board, choosing a move,
  planning the grasp, moving, checking. Gemma, running locally, comments on the robot's moves and
  helps when the camera can't tell what you played.
- **Look under the hood.** One click opens a Rerun debug view with camera feeds, joint trajectories and
  game state on a shared timeline. Recording to disk is one flag away.
- **Runs in simulation.** The whole stack plays full games in Gazebo with the same interfaces as the
  real robot.
- **Works on your phone.** Open the UI from any device on your network.

<table>
  <tr>
    <td width="42%"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-top-down.png" alt="Top-down view with the arm drawn translucent over the board"/></td>
    <td width="20%"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-phone.png" alt="The UI on a phone"/></td>
    <td width="38%"><img src="https://github.com/Yadunund/chessbot/raw/media/rerun.png" alt="Rerun debug view with cameras, game state and joint plots"/></td>
  </tr>
  <tr>
    <td align="center"><sub>Top down: the arm stays visible, translucent</sub></td>
    <td align="center"><sub>On a phone</sub></td>
    <td align="center"><sub>Rerun debug view</sub></td>
  </tr>
</table>

## Quick start

Runs on Linux (x86-64) with [pixi](https://pixi.sh). Simulation needs no hardware.

```bash
pixi run build          # fetch pinned sources and build the workspace
pixi run stockfish      # build the Stockfish engine

pixi run router         # terminal 1: message router
pixi run -e llm llm     # terminal 2 (optional): Gemma 4 reasoning model on the GPU
pixi run sim            # terminal 3: simulated robot, every chessbot component, Rerun viewer on :9090
```

On the real SO-101, run `pixi run real` instead of `pixi run sim`, then press **Set up…** in the UI and
follow the four steps: the arm shows the camera its own gripper, you click the board's corners, and it
tells you whether every square is in reach (see [hardware setup](docs/hardware.md)). Simulation and the
real robot are the same launch file, chosen with `hardware:=`.

Open **`http://<host>:8000`**, choose your colour and strength, and press **New game**.

| To… | Run |
|---|---|
| Record the session to `~/chessbot_recordings` | `pixi run sim record:=true` |
| Use the desktop Rerun viewer instead of the web one | `pixi run sim viewer:=native` |
| Check every connection in the stack | `pixi run check` |
| Also play a short game automatically | `pixi run check --e2e --moves 4` |
| Stop everything | `tools/dev/stop.sh` (add `--all` to stop the router) |
| Drive the arm by hand | The **Robot** card: jog the tool, work the jaws, send it home |

## Playing

1. **New game.** Pick a colour and a strength. The robot sets its clock and, if it plays white, moves first.
2. **Your move.** Move a piece on the board and press the clock. The robot reads the move from its camera;
   if it can't tell, it asks you to type it (for example `e2e4`).
3. **The robot's move.** It checks your move is legal, chooses a reply, and moves the piece, putting
   captured pieces beside the board. Follow along in the 3D view and the thought feed.
4. **If something goes wrong**, it stops and asks for help. Fix the board and press **Resume**.

## Status

The whole application runs in simulation: the robot reads your move from the camera (three moves out of
three in the last run), replies, moves the pieces and comments with Gemma.

On the real SO-101 the stack comes up end to end - driver, controllers, live joint states, the overhead
camera streaming to Rerun - and the arm takes commanded moves. It has not picked a piece yet: that needs
the setup workflow run on the rig first, since an uncalibrated arm does not know where the board is.

Gemma's advice is only accepted when it can be shown to be reading the image: it is asked a decoy
question about the same frame, and an answer that survives both is trusted. On the real board, in poor
light, it did not survive, which is the point of asking.

The motion scenarios (`pixi run sim-test`) pass 13 cases, including castling, the edge and far
files, and full robot turns with a capture, placing pieces within 0.8 mm. En passant and
promotion still fail, and both now report in seconds rather than hanging.

Next: tuning motion against the real arm, move-reading accuracy, en passant and promotion.

## Learn more

- [Design](docs/design.md): architecture, components and how data flows
  reusable parts of it
- [Hardware](docs/hardware.md): running on the real SO-101
- [Interfaces](docs/interfaces.md): every boundary and its type
- [Skills](src/chessbot_brain/skills/README.md): the robot's actions and their contracts
- [Licensing](docs/licensing.md): dependency policy

## License

Apache-2.0. Stockfish (GPL-3.0) is built separately and used only as an external process.

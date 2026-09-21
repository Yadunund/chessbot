<h1 align="center">chessbot</h1>

<p align="center">
  <b>Play chess against a real robot arm, on a real board.</b><br/>
  Move your piece, press the clock, and watch the robot look, think and reply.
</p>

<p align="center">
  <img src="https://github.com/Yadunund/chessbot/raw/media/ui-orbit.gif" alt="The chessbot UI orbiting its 3D view: the SO-101 arm and the board it believes in" width="620"/>
</p>

<p align="center">
  <sub>What the robot believes, drawn live — the arm moves in the browser as it moves on the table.</sub>
</p>

## What it does

- **A robot opponent that moves the pieces.** An [SO-101](https://github.com/TheRobotStudio/SO-ARM100)
  arm picks and places its own pieces, captures included. Stockfish picks its moves at the strength you choose.
- **It reads your move from a camera.** No board sensors, no special pieces. An overhead webcam,
  and the move is worked out from what changed.
- **It measures its own board.** You guide the gripper to the four corners; the arm works out where the
  board is, how big its squares are and how high it sits, from its own kinematics.
- **It narrates.** A thought feed shows each step — reading the board, choosing, planning the grasp,
  moving, checking. Gemma runs locally and comments, and helps when the camera is unsure.
- **It runs with no hardware at all.** The whole stack plays full games in Gazebo through the same
  interfaces as the real arm.

<table>
  <tr>
    <td width="46%" valign="top"><img src="https://github.com/Yadunund/chessbot/raw/media/real-rig.jpg" alt="The physical rig: an SO-101 arm beside a wooden chessboard, with a webcam on a mast above"/></td>
    <td width="54%" valign="top"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-3d.png" alt="The chessbot web UI, mid-game"/></td>
  </tr>
  <tr>
    <td align="center"><sub>The rig: an arm, a board, and one webcam on a mast</sub></td>
    <td align="center"><sub>The UI: board, clocks, and the robot's own view of the world</sub></td>
  </tr>
</table>

## Quick start (no robot needed)

Linux (x86-64) with [pixi](https://pixi.sh). Simulation needs no hardware.

```bash
pixi run import-deps && pixi run build && pixi run stockfish   # once
```

Then, each in its own terminal:

```bash
pixi run router     # the message router — exactly one, ever
pixi run sim        # the simulated robot and every chessbot component
```

Open **`http://localhost:8000`**, press **New game**, and play. That's it.

> Optional: `pixi run -e llm llm` adds the local Gemma model for commentary and tie-breaks.

## On the real SO-101

```bash
pixi run real usb_port:=/dev/ttyACM0 overhead_device:=/dev/video0 wrist_camera:=false
```

Simulation and the real robot are the same launch file, chosen with `hardware:=`. RViz opens
alongside it wherever there is a display (`rviz:=false` to skip), showing the arm, its frames and the
board once it is measured.

### Calibrating: two measurements, in this order

The robot has to know where the board is before it can touch a piece. Press **Set up…** in the UI.

<table>
  <tr>
    <td width="52%" valign="top">

**1. Measure the board** — *only when the board has moved*

1. Press **Release arm**. The motors let go, so you can move it by hand.
2. Close the jaws and rest **their tips** on the outer corner of **a1**, then press **Confirm touch**.
3. Repeat for **h1**, **h8**, **a8** — going round the board, with a1 to the left of the white pieces.
4. Press **Reactivate arm**.

The arm now knows the board's position, its square size and how high it sits, all from its own
kinematics — no camera involved, nothing to type. If you know your square size, type it in and it is
checked against the measurement.

**2. Place the camera** — *only when the camera has moved*

Click the same four corners in the photo, in the same order. Paired with where the arm found those
corners, that is where the camera is, which is how each pixel is assigned to a square. The arm does
not depend on this; perception does.

**3. Save.** Nothing takes effect until you press **Save calibration**.

  </td>
    <td width="48%" valign="top"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-calibration.png" alt="The setup dialog: touch the four corners, then click them in the camera photo"/></td>
  </tr>
</table>

Moved only the camera? Do step 2 alone — the corners are still where the arm last measured them.

## Playing

1. **New game.** Pick your colour, the robot's strength and a clock.
2. **Your move.** Move a piece, then press the clock. The robot reads the move from its camera; if it
   cannot tell, it asks you to type it (`e2e4`).
3. **The robot's move.** It checks yours is legal, picks a reply and moves the piece, putting captures
   beside the board.
4. **If something goes wrong** it stops and asks for help. Fix the board and press **Resume**, which
   hands back to whoever actually has the move.

<table>
  <tr>
    <td width="34%"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-top-down.png" alt="Top-down view with the arm drawn translucent over the board"/></td>
    <td width="16%"><img src="https://github.com/Yadunund/chessbot/raw/media/ui-phone.png" alt="The UI on a phone"/></td>
    <td width="50%"><img src="https://github.com/Yadunund/chessbot/raw/media/rerun.png" alt="Rerun debug view with cameras, game state and joint plots"/></td>
  </tr>
  <tr>
    <td align="center"><sub>Top down, arm translucent</sub></td>
    <td align="center"><sub>On a phone</sub></td>
    <td align="center"><sub>Rerun: cameras, state and joints on one timeline</sub></td>
  </tr>
</table>

| To… | Run |
|---|---|
| Report moves without moving the arm | `pixi run real play_moves:=false` |
| Record the session to `~/chessbot_recordings` | `pixi run sim record:=true` |
| Use the desktop Rerun viewer | `pixi run sim viewer:=native` |
| Check every connection in the stack | `pixi run check` |
| Play a short game automatically | `pixi run check --e2e --moves 4` |
| Check the camera reads moves, in simulation | `pixi run python tools/sim/perception_check.py --rounds 20` |
| Stop everything | `tools/dev/stop.sh` (`--all` also stops the router) |
| Find the arm on its bus, before blaming the driver | `pixi run python tools/dev/ping_servos.py /dev/ttyACM0` |
| See what is holding a serial port | `fuser -v /dev/ttyACM0` |

## Status

**On the real arm.** It measures its own board by touch: on a board whose squares are a known
25.4 mm, it came back with 25.57 mm. It reads human moves from the camera — *"I saw b1c3: it explains
the changes on b1, c3"* — and it completes a pick and place, descend, grasp, lift, traverse, place
and park.

**In simulation.** Full games: the robot reads the move, replies, moves the pieces and comments. The
motion suite (`pixi run sim-test`) passes 13 scenarios including castling, the far files, captures and
full turns, placing within 0.8 mm. En passant and promotion still fail.

**Known rough edges.** The camera fit is taken from four corners that lie in a plane, which constrains
it weakly, so treat the reported error as decorative; touching each corner at two heights would fix it.
The servos hold a few degrees short of a commanded pose under load, which the grasp height currently
absorbs. `tools/sim/perception_check.py` does not pass yet in simulation.

## Learn more

- [Design](docs/design.md): architecture, components and how data flows
- [Hardware](docs/hardware.md): running on the real SO-101
- [Interfaces](docs/interfaces.md): every boundary and its type
- [Skills](src/chessbot_brain/skills/README.md): the robot's actions and their contracts
- [Agent skills](.claude/skills): bringing a rig up, calibrating without markers, guarding a model
  that guesses, measuring in simulation
- [Licensing](docs/licensing.md): dependency policy

## License

Apache-2.0. Stockfish (GPL-3.0) is built separately and used only as an external process.

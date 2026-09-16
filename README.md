# chessbot

Play chess against a real robot arm on a physical board.

A robot arm (starting with the [SO-101](https://github.com/TheRobotStudio/SO-ARM100)) watches the board with an overhead and a wrist camera, plans its reply, and moves the pieces itself. You play through a minimal web page on the same network: press the big clock button when you've moved.

> **Status:** the full stack runs in simulation. Stockfish plays the robot's moves and checks yours for legality. The arm picks and places in Gazebo, and the UI and Rerun debug view are live. Not yet implemented: detecting your move from the camera (the clock press takes the move explicitly for now), a board and pieces in simulation, and the calibration measurement itself.

## Quick start (linux-64)

Requires [pixi](https://pixi.sh).

```bash
pixi run build          # imports external sources and builds the workspace
pixi run stockfish      # builds the Stockfish engine from pinned upstream source

pixi run router         # terminal 1: Zenoh router (ROS middleware + REST for the browser)
pixi run sim            # terminal 2: Gazebo, the robot and every chessbot component
                        #   add record:=true to also archive the Rerun stream to ~/chessbot_recordings
pixi run viewer         # terminal 3 (optional): Rerun web viewer on :9090
```

Then open `http://<host>:8000`: the UI, served by the brain.

To check that everything is wired up:

```bash
pixi run check                     # every interface boundary
pixi run check --e2e --moves 4     # also plays a short game in simulation
```

To stop the stack: `tools/dev/stop.sh` (add `--all` to stop the router too).

## Architecture

The **brain** is the game engine and orchestrator. It composes capabilities as a client, publishes game state and a readable thought feed, and serves a REST API and the UI. Each **capability** sits behind a contract, so implementations can be swapped:

| Package | Role |
|---|---|
| `chessbot_brain` | game engine, orchestrator, skills ([docs](src/chessbot_brain/skills/README.md)), REST API |
| `chessbot_perception` | camera frames → board facts, on request (C++ component) |
| `chessbot_motion` | IK and Cartesian paths behind MoveIt's standard services |
| `chessbot_calibration` | calibration YAML, Zenoh key-value store, `Calibrate` action |
| `rerun_ros_bridge` | generic ROS 2 → Rerun bridge (C++ component, no chessbot dependencies) |
| `chessbot_interfaces` | the few custom interfaces: game state, thoughts, board state, calibration |
| `chessbot_description` | SO-101 with cameras, ros2_control, simulation world |
| `chessbot_bringup` | launch files and configuration |
| `chessbot_web` | the UI: plain HTML, CSS and JavaScript |

Camera frames, perception and the Rerun bridge share one C++ container (`robot_io`) with intra-process comms and the callback-group events executor, so images are never serialised between them.

See [docs/interfaces.md](docs/interfaces.md) for every boundary and its type.

## Layout

| Path | Contents |
|---|---|
| `src/` | ROS 2 packages |
| `config/zenoh/` | router configuration |
| `tools/` | reachability analysis, stack checks, dev scripts |
| `docs/` | interfaces, licensing |
| `chessbot.repos` | pinned external sources (SO-101 description, Stockfish) |

## License

Apache-2.0. All dependencies must be Apache-2.0 compatible; see [docs/licensing.md](docs/licensing.md). Stockfish (GPL-3.0) is built separately and used only as an external process over UCI.

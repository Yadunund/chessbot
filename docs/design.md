# Design

How chessbot is put together, and why. For every interface and its exact type, see
[interfaces.md](interfaces.md). For skill contracts, see
[skills](../src/chessbot_brain/skills/README.md).

## Goals

- **Play a real game** against a human on a physical board, with a low-cost arm (SO-101 first).
- **Any arm, any board.** Hardware sits behind ros2_control, and the board is described by
  calibration. A different arm or board changes configuration, not code (within the arm's reach).
- **Swappable capabilities.** Perception, motion planning, control and calibration each sit
  behind a contract. A better implementation (MoveIt, a learned grasp policy, a VLM) drops in
  without changing the brain.
- **Observable by default.** Every decision is narrated, every sensor stream can be recorded,
  and the UI shows what the robot believes.
- **A data flywheel.** Recorded games become training data for perception and, later, learned
  manipulation policies.
- **Simulation first.** The whole stack runs against Gazebo with the same interfaces as the
  real robot.

## Principles

1. **Contracts before code.** Interfaces are agreed from requirements before implementation.
   Standard ROS interfaces are reused wherever they exist (`moveit_msgs`, `control_msgs`,
   `sensor_msgs`). Custom ones exist only for game state and perception, are minimal, and
   use enums rather than strings.
2. **Observations are events; inference is a service; motion is an action.** Nothing polls for
   state. Expensive inference (reading the board) runs only on request.
3. **One writer per piece of state.** The brain owns the game. The calibration node owns the
   calibration. Everyone else reads.
4. **Heavy data stays in C++ and in one process.** Camera frames, perception and the Rerun
   bridge are composed into one container with intra-process communication, so images are
   passed by pointer, never serialised. C++ nodes with many callbacks use the callback-group
   events executor.
5. **No extra bridges.** The browser reads ROS topics through the Zenoh router the middleware
   already uses. Visualisation reuses Rerun's own viewer.
6. **Safety is hardware.** A physical stop cuts servo power. Software pause and park are
   ordinary operations, not safety functions.
7. **Permissive licences only.** Everything linked, vendored or shipped is Apache-2.0
   compatible. Stockfish (GPL) runs as a separate process over UCI. See [licensing.md](licensing.md).

## System overview

```mermaid
flowchart LR
  subgraph browser["Browser (phone or laptop)"]
    UI["Chessbot UI<br/>3D belief view · clock · thoughts"]
    RV["Rerun web viewer<br/>(debug view)"]
  end

  subgraph host["Robot host"]
    ZR["Zenoh router<br/>ROS middleware · REST/SSE · key-value store"]
    subgraph robot_io["robot_io container (C++, intra-process)"]
      CAM["cameras<br/>(Gazebo bridge in sim)"]
      PER["perception<br/>board facts on request"]
      RRB["rerun_ros_bridge<br/>gRPC stream · optional .rrd"]
    end
    RC["ros2_control<br/>arm + gripper controllers"]
    RSP["robot_state_publisher"]
    MOT["motion<br/>IK · Cartesian paths"]
    CAL["calibration<br/>YAML ↔ key-value store"]
    BRAIN["brain<br/>game engine · skills · REST API"]
    SF["Stockfish<br/>(UCI process)"]
  end

  CAM --> PER & RRB
  RC -- joint states --> RRB
  BRAIN -- "GetBoardState" --> PER
  BRAIN -- "IK / Cartesian path" --> MOT
  BRAIN -- "trajectory / gripper actions" --> RC
  BRAIN -- "Calibrate action" --> CAL
  CAL -- "calibration JSON" --> ZR
  BRAIN <-- UCI --> SF
  UI -- "REST: commands, snapshot, URDF" --> BRAIN
  UI -- "SSE: game state, thoughts, joint states<br/>GET: calibration" --> ZR
  RV -- gRPC --> RRB
```

All ROS traffic goes through the Zenoh router (`rmw_zenoh`). The router also runs the REST
plugin, so the browser can subscribe to any topic over Server-Sent Events, and an in-memory
storage that acts as a shared key-value store.

## Components

| Component | Package | Language | Role |
|---|---|---|---|
| Brain | `chessbot_brain` | Python | Owns the game: position, moves, clocks, phase. Composes capabilities into skills. Publishes game state and a readable thought feed. Serves the REST API and the UI. |
| Perception | `chessbot_perception` | C++ | Turns camera frames into board facts (`GetBoardState`), only when asked. Board detection is not implemented yet. |
| Motion | `chessbot_motion` | Python | IK and Cartesian paths behind MoveIt's standard services, using Pinocchio. |
| Control | ros2_control | C++ | `joint_trajectory_controller` for the arm, `parallel_gripper_action_controller` for the gripper, `joint_state_broadcaster`. Gazebo, mock or real hardware behind the same controllers. |
| Calibration | `chessbot_calibration` | Python | Where the board is relative to the robot. Loads YAML from disk, primes the key-value store, runs the `Calibrate` action (measurement stages are stubs). |
| Recording | [`rerun_ros_bridge`](https://github.com/Yadunund/rerun_ros_bridge) (external) | C++ | Generic ROS 2 → Rerun bridge in its own repository, pinned in `chessbot.repos`. Streams to the Rerun viewer over gRPC, and archives to `.rrd` when asked. |
| UI | `chessbot_web` | JS | Plain HTML, CSS and JavaScript, no build step. three.js for the 3D view. |
| Description | `chessbot_description` | URDF | SO-101 with overhead and wrist cameras, ros2_control tags, simulation world. |
| Bringup | `chessbot_bringup` | XML launch | Launch files and configuration. |
| Interfaces | `chessbot_interfaces` | IDL | `GameState`, `Thought`, `BoardState`, `GetBoardState`, `Calibrate`. |

## The brain

The brain is a game engine and an orchestrator in one process. It is a **client** of every
capability, and exposes only its own state.

- **Rules and moves.** Stockfish runs as a separate process. The brain asks it for legal moves
  (`go perft 1`), check (`d`) and its reply (`go movetime`, with `UCI_Elo` for strength).
  Board bookkeeping (FEN, castling, en passant, promotion effects) is our own code.
- **Phases.** `idle → setup → human_turn → reading_board → thinking → moving → verifying →
  human_turn`, with `needs_help`, `calibrating` and `game_over`. One job runs at a time;
  commands during a job are rejected with HTTP 409.
- **Thought feed.** Every step is narrated on `/chessbot/thoughts` (`perceive`, `decide`, `plan`,
  `act`, `verify`, `recover`, `explain`), which the UI shows and Rerun records.
- **Geometry.** Squares and graveyard slots become robot-frame positions from the calibration
  profile, read from the key-value store. Nothing else about the board is hard-coded.

### A turn

```mermaid
sequenceDiagram
  actor Human
  participant UI
  participant Brain
  participant Perception
  participant Stockfish
  participant Motion
  participant Controllers

  Human->>Human: moves a piece
  Human->>UI: presses the clock
  UI->>Brain: POST /api/press_clock {move}
  Brain->>Stockfish: legal moves (perft)
  Brain->>Perception: GetBoardState
  Brain->>Stockfish: best reply
  loop each pick and place
    Brain->>Motion: GetPositionIK (hover)
    Brain->>Motion: GetCartesianPath (descend / lift)
    Brain->>Controllers: FollowJointTrajectory
    Brain->>Controllers: ParallelGripperCommand
  end
  Brain->>Perception: GetBoardState (verify)
  Brain-->>UI: game state and thoughts (via router SSE)
```

Until camera move detection exists, the clock press carries the human's move explicitly.

### Skills

A skill is a sequence composed from capabilities that does one physical thing and returns or
raises. Skills keep no state and never wait on a human; the game logic decides what a failure
means. Each has a `SKILL.md` contract in the Agent Skills format, so a reasoning model can load
them later.

`execute_chess_move` → `transfer` (captures to the graveyard first, then the move, then the
castling rook) → `pick` / `place` → hover above the target at the highest reachable transit
height, straight-line descent, gripper, straight-line lift.

Skills are composed in code today. The same contracts allow a behaviour tree, or code written
by a reasoning model, to compose them later.

## Calibration and the key-value store

The calibration profile says where the board is in the robot's base frame: the a1 outer
corner, the board's yaw, and the square size.

- The calibration node is the only writer. On startup it loads `~/.chessbot/calibration.yaml`
  (or writes nominal values) and puts it into the router's key-value store as JSON under
  `chessbot/kv/calibration`.
- Every reader (the brain, the UI) gets it from the store, never from disk.
- A new calibration overwrites both the YAML and the stored value. The profile carries a
  `version`, so a reader can reject an incompatible one.

The nominal placement puts the robot at the rank-8 edge (it plays black, the human sits
opposite), with the near edge 10 cm from the pan axis. That keeps every square and graveyard
slot reachable with a near-vertical approach; `tools/dev/board_reach.py` checks it against
the real IK.

## Motion

- **IK** solves for gripper position plus approach direction with bounded least squares over
  the joint limits (Pinocchio forward kinematics, SciPy solver). Seeds come from sampled
  configurations; if a strictly vertical approach fails, the tilt tolerance relaxes to 25°.
- **Cartesian paths** interpolate straight lines and solve IK along them, seeded by the
  previous point.
- Both are served through `moveit_msgs` services, so MoveIt could replace this node without
  changing the brain.

## Seeing the robot's belief

The UI renders the brain's **belief world**, not camera images: what the robot thinks is on
the board, and where its arm is.

| What is drawn | Where it comes from |
|---|---|
| Board | calibration profile, read from the router's key-value store |
| Pieces | FEN from the game state |
| Captured pieces | the brain's graveyard, with each piece's slot |
| Arm | URDF and meshes served by the brain, posed by live `/joint_states` |

The same data drives three views: **3D** from the human's seat, **top down** with the arm drawn
translucent, and a flat **2D** board. The URDF loader is ours; rendering uses three.js.

When camera perception is implemented, squares where the camera disagrees with the belief can
be highlighted in the same view.

### How the browser gets data

No extra bridge process is involved:

- **Commands and snapshots:** the brain's REST API.
- **Live topics:** the Zenoh router's REST plugin streams any topic as Server-Sent Events. The
  UI decodes the CDR payloads itself (`cdr.js`) for `GameState`, `Thought` and `JointState`.
- **Calibration:** a plain HTTP GET on the key-value store.
- **Writes** go through the brain's REST API, not the router: `rmw_zenoh` ignores samples
  without its attachment metadata.

## Recording and visualisation

[`rerun_ros_bridge`](https://github.com/Yadunund/rerun_ros_bridge) lives in its own repository, since nothing in it is chessbot-specific; its README describes its architecture. It runs in the same container as the cameras, so frames reach it by pointer.

- It serves a gRPC stream that the stock Rerun web or native viewer connects to. `sim.launch.xml`
  starts the viewer through the bridge's `viewer.launch.xml` (`viewer:=web|native|none`), and the
  UI's **Debug view** button opens the web viewer in a new tab.
- It logs the robot model (URDF through Rerun's importer, moved by `/tf`), images (with encoding
  and row-padding handling and optional downscaling), joint states, and any other topic as text
  (type discovered at runtime, rendered as YAML through ROS introspection).
- The layout is `config/chessbot.rbl`, generated by `src/chessbot_bringup/rerun/make_blueprint.py`
  (`pixi run rerun-blueprint`): the 3D scene takes most of the screen, with cameras, joint plots,
  thoughts and game state beside it.
- Everything is logged on one `ros_time` timeline from the node's clock: simulation time when
  `use_sim_time` is set, wall-clock time otherwise.
- Archiving to `.rrd` is off unless requested: `pixi run sim record:=true`.

Rerun recordings are the start of the data flywheel: paired camera frames, joint trajectories
and narrated skill calls.

## Simulation

Gazebo (gz sim) runs headless inside the `robot_io` container, with the ros_gz bridge as
another component. The robot uses `gz_ros2_control` with the same controllers as real
hardware, and the overhead and wrist cameras are simulated sensors. The simulation does not
include a board or pieces yet, so perception reports no board and the brain's belief is the
only board.

`pixi run check` exercises every interface boundary. With `--e2e` it plays a game in simulation,
including an illegal move and captures.

## Deployment and compute

- Prototyping runs on one Linux desktop. The Pixi workspace targets linux-64, linux-aarch64 and
  osx-arm64.
- Any node can run on another machine by connecting to the same router, for example GPU
  inference on a separate box. That needs configuration, not code.
- Moving to smaller compute later is an optimisation, not a design constraint.

## Status and roadmap

**Working in simulation:** full games against Stockfish with legality checks; pick and place
across every square and graveyard slot, including captures and castling; the 3D belief UI;
the thought feed; the Rerun debug view.

**Next:**

- a board and pieces in simulation
- camera move detection, triggered by the clock press
- calibration measurement
- promotion piece swaps
- a vision-language model for reasoning, cheat checks and board disagreements
- a real SO-101 on the same stack
- recorded episodes toward learned manipulation

## Repository layout

| Path | Contents |
|---|---|
| `src/` | ROS 2 packages |
| `config/zenoh/` | router configuration |
| `tools/` | stack checks, reachability analysis, dev scripts |
| `docs/` | design, interfaces, licensing |
| `media` branch | README screenshots and GIFs, kept out of `main`'s history |
| `chessbot.repos` | pinned external sources (rerun_ros_bridge, SO-101 description, Stockfish), imported into `external/` |

# Interfaces

Every boundary in the stack, who owns it, and why it has the shape it has.
The brain is a client of every capability, and serves only its own state and a
REST API.

Custom interfaces exist only where the ROS ecosystem has none: game state and
perception outputs (`chessbot_interfaces`). Everything else reuses standard
messages, so a component can be swapped for any implementation of the same
contract. For example, `chessbot_motion` could be replaced by MoveIt.

## Map

| Boundary | Interface | Type | Owner (server / publisher) | Consumers |
|---|---|---|---|---|
| Camera frames | `/overhead_camera/image_raw`, `/wrist_camera/image_raw` (+ `camera_info`) | `sensor_msgs/Image` topic | camera driver (ros_gz bridge in sim) | perception, Rerun bridge |
| Robot model | `/robot_description` | `std_msgs/String` topic, latched | robot_state_publisher | motion |
| Joint state | `/joint_states` | `sensor_msgs/JointState` topic | ros2_control `joint_state_broadcaster` | motion, brain, Rerun bridge |
| Transforms | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | anyone |
| Board facts | `/perception/get_board_state` | `chessbot_interfaces/srv/GetBoardState` | `chessbot_perception` (C++, in `robot_io`) | brain |
| IK | `/compute_ik` | `moveit_msgs/srv/GetPositionIK` | `chessbot_motion` | brain |
| Cartesian paths | `/compute_cartesian_path` | `moveit_msgs/srv/GetCartesianPath` | `chessbot_motion` | brain |
| Arm motion | `/arm_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | ros2_control `joint_trajectory_controller` | brain |
| Gripper | `/gripper_controller/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | ros2_control `parallel_gripper_action_controller` | brain |
| Calibration run | `/calibration/calibrate` | `chessbot_interfaces/action/Calibrate` | `chessbot_calibration` | brain, UI (via brain) |
| Calibration data | Zenoh key `chessbot/kv/calibration` | JSON (`version` field), in-memory router storage | `chessbot_calibration` primes it at startup | brain, UI (HTTP GET) |
| Game state | `/chessbot/game_state` | `chessbot_interfaces/msg/GameState` topic, latched | brain | UI (SSE), recorder |
| Reasoning | `/chessbot/thoughts` | `chessbot_interfaces/msg/Thought` topic | brain | UI (SSE), recorder |
| UI commands + snapshot | `http://<host>:8000/api/*` | REST (JSON) | brain | UI, scripts |
| Browser read path | `http://<host>:8080/0/<topic>/**` | Zenoh REST plugin, Server-Sent Events, base64 CDR | Zenoh router | UI |
| Visualisation | `rerun+http://<host>:9876/proxy` | Rerun gRPC | `rerun_ros_bridge` (C++, in `robot_io`) | Rerun viewer |

## Brain REST API

| Method | Path | Body | Effect |
|---|---|---|---|
| GET | `/api/state` | — | snapshot: phase, FEN, moves, clocks, recent thoughts, capability availability, last error |
| POST | `/api/new_game` | `{"robot_side": "black"\|"white", "engine_elo": 0}` | reset the game; the robot moves first if white |
| POST | `/api/press_clock` | `{"move": "e2e4"}` (optional) | end the human's turn. Until camera move detection exists, the move must be given |
| POST | `/api/calibrate` | — | run the calibration action |
| POST | `/api/park` | — | park the arm |
| POST | `/api/resume` | — | leave `needs_help` and hand the turn to the human |
| POST | `/api/demo_transfer` | `{"src": "e2", "dst": "e4"}` | dev: move a piece between squares |

Jobs run one at a time. A command arriving while one runs gets HTTP 409.

## Conventions
- **Squares** are indexed `file + 8 * rank` (a1 = 0, h8 = 63) wherever an array is used.
- **Board frame:** origin at the outer corner of a1 on the playing surface, +x towards the h-file, +y towards rank 8, +z up. Its pose in `base_link` comes from calibration.
- **Motion targets:** the tool's approach direction is the target pose's +Z axis. `(1, 0, 0, 0)` points it straight down. Roll about that axis is free (5-DoF arm).
- **Perception answers** carry the header (stamp and frame) of the camera frame they came from.

## Known issue
On Lyrical with rmw_zenoh, a C++ service created in a separately created callback group was never executed, while subscriptions in such groups work. `chessbot_perception` therefore keeps its service in the default group.

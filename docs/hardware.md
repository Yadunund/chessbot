# Real hardware

Runs on Linux (x86-64 or arm64) with the SO-101 and at least an overhead USB camera.

## 1. Find the arm

The servo bus turns up as a USB serial device. Before involving the stack, check the servos
answer:

```bash
pixi run python tools/dev/ping_servos.py /dev/ttyACM0 /dev/ttyACM1
# /dev/ttyACM0 at 1000000: 1: 0.2 ms, 2: 0.2 ms, ... 6: 0.2 ms
```

Six replies at 1 Mbaud means the arm is on that port, powered and awake. No reply means the
wrong port, the wrong baud rate, or no bus power.

If the port is there but the stack cannot open it, something else holds it:

```bash
fuser -v /dev/ttyACM0     # a LeRobot session, or an orphaned controller manager
```

Stable names are worth setting up once:
[`99-chessbot.rules.example`](../src/chessbot_bringup/config/hardware/99-chessbot.rules.example)
creates `/dev/so101`, `/dev/cam_overhead` and `/dev/cam_wrist`.

## 2. Servo calibration

Calibrate the arm with LeRobot's procedure, which writes each joint's homing offset into the
servo's own EEPROM. That is where the calibration lives and the stack needs nothing further:
leave `joint_config_file` empty (the default).

Only pass a `joint_config_file` to override servo registers deliberately; a copy of
[`so101_joints.example.yaml`](../src/chessbot_description/config/hardware/so101_joints.example.yaml)
at `~/.chessbot/so101_joints.yaml` is then written to the servos at start-up.

## 3. Run

```bash
pixi run router
pixi run -e llm llm        # optional, on the GPU machine
pixi run real usb_port:=/dev/ttyACM0
```

With only an overhead camera fitted:

```bash
pixi run real usb_port:=/dev/ttyACM0 wrist_camera:=false overhead_device:=/dev/video0
```

`pixi run real hardware:=mock cameras:=false` brings the whole stack up without the arm or
cameras, to check an installation. The real arm runs at a quarter of the simulated speed
(0.4 rad/s, 2 cm/s); raise it with `joint_velocity:=` and `cartesian_speed:=` once motion has
been tuned against your rig.

## 4. Set the rig up, from the UI

The arm and the camera have to agree on where the board is, and nothing about a real rig
supplies that for free. Open the UI (<http://localhost:8000>), press **Set up…**, and work
through the steps:

1. **Find the camera.** The arm moves through a dozen poses and opens its jaws at each. The
   jaws are the only thing that moves between two frames, so differencing them finds the
   gripper in the image, and the camera pose is fitted to where the gripper actually was.
   Keep the workspace clear while this runs. The step reports its reprojection error.
2. **Mark the board.** Click the board's four outer corners - a1, h1, h8, a8 - in the photo.
   Those pixels back-project onto the table, which gives the board's origin, rotation and
   square size. The reported *squareness* is the check: a large value means a corner was
   mis-clicked or the camera fit is wrong.
3. **Choose where it waits.** Jog the arm with the Robot controls to somewhere clear of the
   board and out of the camera's view, then press **Park here**.
4. **Save.** The calibration node writes the profile and primes the key-value store that
   perception and motion both read.

Before saving, the workflow also checks every square against IK and tells you how many are out
of reach. The SO-101 is short: a board much further than about 25 cm from the base, or turned
away from it, will not be fully reachable, and the fix is to move the board rather than to let
the arm strain at it.

Until this is done, **do not ask the robot to pick pieces**. It does not know where the board
is, and an uncalibrated grasp attempt drives the gripper into the board.

## 5. Driving the arm by hand

The Robot card jogs the tool 5, 10 or 20 mm at a time along each axis, opens and closes the
jaws, and shows the tool's position. Every target is clamped into a box above the table and
within reach (`JOG_BOX` in the brain), because an uncalibrated rig has no idea where the board
is.

## Lighting

Perception classifies colours, so the board has to be lit evenly enough that the light pieces
read as light everywhere on it. A board half in shadow defeats both the colour classifier and
the vision model - the model will still answer, confidently, and be wrong. Fix the lighting
before trusting anything either of them says.

## Cameras

Camera intrinsics are placeholders
([`overhead_intrinsics.yaml`](../src/chessbot_bringup/config/cameras/overhead_intrinsics.yaml));
replace them with a calibration for your camera (`ros2 run camera_calibration
cameracalibrator`). Until then the setup workflow assumes a Logitech C920's focal length.

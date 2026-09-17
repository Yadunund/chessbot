# Real hardware

Runs on Linux (x86-64 or arm64) with the SO-101 and two USB cameras connected.

## 1. Devices

Give the arm and cameras stable names with udev rules:
[`chessbot_bringup/config/hardware/99-chessbot.rules.example`](../src/chessbot_bringup/config/hardware/99-chessbot.rules.example)
creates `/dev/so101`, `/dev/cam_overhead` and `/dev/cam_wrist`. Follow the steps in the file.

## 2. Servo calibration

Copy [`so101_joints.example.yaml`](../src/chessbot_description/config/hardware/so101_joints.example.yaml)
to `~/.chessbot/so101_joints.yaml` and set each joint's `homing_offset`, `range_min` and `range_max`
for your arm (LeRobot's calibration gives these values).

## 3. Overhead camera

Mount the camera about 50 cm above the board centre. Pass its pose relative to the robot's base if it
differs from the default (`0.234 0 0.51`, looking straight down):

```bash
pixi run real overhead_xyz:="0.234 0.0 0.51" overhead_rpy:="3.14159 0.0 -1.5708"
```

Camera intrinsics are placeholders
([`overhead_intrinsics.yaml`](../src/chessbot_bringup/config/cameras/overhead_intrinsics.yaml)); replace
them with a calibration for your camera (`ros2 run camera_calibration cameracalibrator`).

## 4. Run

```bash
pixi run router
pixi run -e llm llm        # optional, on the GPU machine
pixi run real
```

`pixi run real hardware:=mock cameras:=false` brings up the whole stack without the arm or cameras,
to check an installation.

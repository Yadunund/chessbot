---
name: rig-bringup
description: Bring an unfamiliar robot rig up for the first time - find the hardware on its bus, get the driver loading, prove joint states and camera frames flow, and establish a safe envelope before anything moves. Use when connecting real hardware to a stack that so far only ran in simulation.
---

# Bringing a real rig up

Simulation tells you the application works. It tells you nothing about the rig: which port the
arm is on, whether the driver loads at all, where the camera points, or what is within reach.
Work through this in order, and do not skip ahead to motion.

The rule that matters: **on a rig that is not calibrated yet, the robot does not know where
anything is.** Every command must be small, slow, and inside a box you chose deliberately.

## 1. Find the hardware, read-only

Before any driver, prove the device answers.

- Serial bus: ping each id at each plausible baud rate and report reply latency. A tool like
  `tools/dev/ping_servos.py` answers "is the arm on this port, is it powered, is the baud
  right" in one command, without writing anything.
- Cameras: enumerate devices and confirm the process can actually open one. Permissions here
  are often ACLs on the seat's own session, so what works in a desktop terminal can fail over
  ssh.

**Check who already holds the device.** `fuser -v /dev/ttyACM0`. A teleoperation session, or
your own orphaned process from the last attempt, presents exactly as a broken driver: read
timeouts, `Bad file descriptor`, or a device that answers a ping but not the stack. Make the
stack's stop script kill the process that owns the bus, or the next run inherits the problem.

## 2. Make the driver load

A hardware plugin that fails to load looks like a hardware fault but is a build problem. The
usual causes are a missing plugin search path and a library that links something it never
calls (in ROS, message packages export Python bindings, and linking one without `--as-needed`
fails at `dlopen` with `undefined symbol: PyLong_FromLong`).

Read the loader's error text literally. "Could not find shared library" is a path; "undefined
symbol" is a link; "Read timeout" is the bus.

## 3. Prove the signals flow, before commanding anything

- Joint states publish, and the values are plausible for how the arm is physically posed.
- Controllers reach `active`.
- Camera frames reach whatever is meant to consume them. Subscriber counts prove wiring;
  a screenshot of the viewer proves the whole path. Do both - a bridge that logs nothing on
  success is not evidence either way.

## 4. Establish the safe envelope

Write down, in the robot's own frame, the box that manual control may command. Clamp every
target into it, in the server, not in the UI. Choose it from reach limits and from what is
physically around the robot, and state why in a comment.

Then probe it without moving: ask IK which poses in the box are reachable. A short arm holding
its tool vertical often cannot reach anywhere near as high as you would assume, and finding
that with a query costs nothing.

## 5. First motion

1. Command the arm's **current** position. This exercises the whole action path and moves
   nothing.
2. Command one joint a few degrees, slowly. Confirm the measured position followed.
3. Only then move the tool, still slowly, one step at a time, checking after each.

Expect the real arm to track worse than the simulated one: servos sag under gravity, so the
tool sits low. Use measured FK (the transform tree), never the commanded pose, whenever a
measurement depends on where the gripper actually is.

## 6. Know what you have disturbed

Photograph the workspace before you start and again after each phase, and compare. An arm that
brushed the board moves it by a few degrees - invisible in a log, obvious in two images side by
side. If you disturbed something the person cares about, say so plainly and stop moving.

## Done when

- The device answers read-only probes, and nothing else holds it.
- Controllers are active and joint states are live.
- Camera frames are visible in the viewer.
- A safe box is written down and enforced in code.
- The arm has executed one slow commanded move and tracked it.

Everything past this - grasping, placing, playing - needs calibration first. See
[hand-eye-calibration](../hand-eye-calibration/SKILL.md).

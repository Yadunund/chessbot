---
name: hand-eye-calibration
description: Work out where a camera is, and where the workspace is, on a rig with no fiducials - by using the robot's own gripper as the calibration target and asking the person for the few facts only they can supply. Use before any task that needs the robot and the camera to agree on a position.
---

# Calibrating a rig that has no markers

Perception and motion have to agree on where things are. On a real rig nothing supplies that
agreement for free: the camera is mounted wherever it fits, the workspace is wherever it was
put down, and there is no checkerboard in sight.

Two sources of truth are always available: **the robot knows where its own gripper is**, and
**a person can point at things in a photograph**. Calibration is the job of turning those into
a transform, in that order.

## Step 1: the camera, from the gripper

The gripper is a calibration target the rig always has. To find it in the image without any
template matching:

1. Move the tool to a sample pose and photograph it with the jaws **closed**.
2. Open the jaws and photograph again.
3. Difference the two frames. The only thing that moved is the moving jaw, so the weighted
   centroid of the changed pixels is the gripper.

Repeat over poses spread across the reachable workspace, then fit a pinhole camera pose to the
(gripper position, pixel) pairs.

What makes or breaks this:

- **Use measured FK for the 3D point**, not the commanded pose. Real servos sag.
- **Fix the focal length** unless the samples span a lot of depth. A short arm's reachable
  poses are nearly coplanar, and a free focal length simply trades against the camera's
  height: the fit looks converged and puts the camera in the wrong place.
- **Do not let a free "offset" parameter absorb the error.** Bound it, or leave it out. An
  unbounded offset will happily run to 100 metres and report a beautiful residual.
- **Sanity-check each sample by how much changed.** A blob far larger than the jaws means
  something else moved - commonly the whole arm flexing as the gripper actuates - and that
  sample's centroid is not the gripper.
- Try every yaw as a starting guess. The mounting orientation is unknown, and the fit is
  otherwise happy in a local minimum.

Report the reprojection error in pixels. A fit you cannot state an error for is not a
measurement.

## Step 2: the workspace, from the person

Back-projecting a pixel onto the table plane needs only the camera pose and the assumption
that the table is flat. So show the person the live frame and ask them to click the corners of
the workspace, in a stated order. That one interaction removes every ambiguity that automatic
detection struggles with: which rectangle is the board, which corner is the origin, which way
is it turned.

From four corners you get the origin, the rotation, and the scale (square size). Report the
**squareness**: how far the four sides are from equal. That single number catches a bad camera
pose, a mis-clicked corner, and a workspace that is not flat, before anything moves.

## Step 3: reachability, before contact

Ask IK for every cell in the workspace at working height and report which ones fail. A
workspace placed out of reach is then a sentence on screen ("12 squares out of reach, move the
board closer") rather than an arm straining against its limits.

## Step 4: the resting pose, from the person

Where the arm waits is a judgement about the room, not a calculation: clear of the workspace,
out of the camera's view, not over anything fragile. Let the person jog the arm there and save
the pose as it stands.

## Make it a guided flow, not a command

Each step above needs either the arm or the person, and they interleave. Put it in the product
as a sequence of steps with a visible result each time - camera fit error, square size and
squareness, unreachable list, saved pose - and one save at the end. A person who can see each
number can tell you which step is wrong; a single "calibrate" button that either works or does
not cannot be debugged by the person who owns the table.

Keep one writer for the stored calibration. Whichever component owns the profile should be the
only one that writes it; the component that took the measurement hands it over.

## Done when

- The camera pose has a stated reprojection error.
- The workspace has origin, rotation, cell size, and a squareness figure.
- Unreachable cells are listed, and there are none (or the person moved the workspace).
- A resting pose is saved.
- Projecting the workspace back onto a fresh photograph lands on the real thing.

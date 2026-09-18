---
name: sim-measurement-harness
description: Turn a simulator into the agent's own feedback loop - build the world from the application's beliefs, measure outcomes over the simulator's transport, and score runs pass/fail. Use when tuning behaviour that is expensive or risky to iterate on real hardware.
---

# Making the simulator answer questions

A simulator you only look at is a demo. A simulator you can query is a measurement instrument,
and it is what lets an agent iterate without a person in the loop.

## Build the world from the application's own belief

Spawn the scene from whatever the application thinks is true - its board state, its
calibration, its object list - rather than from a hand-authored world file. Then the simulated
world and the application's beliefs cannot drift apart, and any mismatch the run reveals is a
real mismatch.

## Measure over the simulator's transport, not from screenshots

Modern simulators expose far more than pictures. Use it:

- **Contact sensors** on the moving links: "did the gripper touch anything it should not" is
  then a message count with timestamps, not an opinion.
- **Entity poses**: how far each object moved, and how far it tilted, between the start and end
  of a run.
- **Clock and real-time factor**: needed to tell "the robot is slow" from "the simulation is
  slow", which look identical in wall-clock time and have completely different fixes.

Where language bindings are not packaged for your runtime, the simulator's command line tool
piped into a parser is entirely sufficient.

## Score every run

A scenario is a named action plus tolerances: what must move, what must not, how far each may
be off, how far a piece may tilt. Print the measured numbers next to the thresholds and a
single PASS or FAIL. A suite of scenarios becomes the regression test, and the same list
becomes the checklist for the real rig.

Report the number, always. "Placement error fell from 3-4.7 mm to under 2 mm" is a result;
"grasping looks better" is not.

## Watch what actually happened, not what was commanded

Record the controller's own reference and feedback during a run, and derive: tracking lag,
overshoot, peak velocity, peak acceleration, and the number of abrupt velocity changes. This
distinguishes a planning problem from a control problem in one measurement:

- lag near zero and jerky motion -> the trajectory is jerky (fix the planner)
- smooth reference, joint slamming to its speed limit -> the plant or the limits (fix control)
- a joint pinned at exactly a limit value -> your solver is riding the joint limit

## Change one thing at a time

Every parameter you change at once is a confound. When two changes go in together and the
result gets worse, you have learned nothing and have to redo both. Measure the baseline, change
one thing, measure again, and write the number in the commit message.

Beware in particular of physics parameters that trade realism for speed: a coarser step can
take the simulation to real time while quietly making grasping unreliable, and the failure
shows up as a mysterious behaviour change rather than as a physics warning.

## Keep the simulation as hard as reality

Take materials, colours and lighting from the real sensor where you can measure them. A
simulated scene that is cleaner than the real one will pass perception tests the rig fails.
If matching reality makes a test fail, that is the test working.

## Done when

- The world is built from the application's beliefs.
- Every run prints measured numbers and a pass/fail.
- A suite runs unattended with one command.
- The measurements distinguish planning from control problems.

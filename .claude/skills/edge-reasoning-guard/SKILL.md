---
name: edge-reasoning-guard
description: Add a local vision-language model as an advisor to a deterministic system, and prove it is actually reading its input rather than answering from priors. Use whenever a model's answer is allowed to change what a robot does.
---

# Letting a model advise, without letting it guess

A small local model behind a schema-constrained service is a cheap way to cover cases the
deterministic path cannot decide. The danger is specific and easy to miss: **a model asked to
choose between plausible options will choose the one its priors favour, at maximum confidence,
without having read the input at all.**

A confident answer is not evidence that the input was read. A model can produce one from its
priors alone, and it will read no differently from one drawn from the input.

## Structure

1. **Deterministic first.** The model is consulted only where the deterministic reader is
   genuinely ambiguous, and it may only choose among options that reader already considers
   legal. It never proposes an action of its own.
2. **Constrain the output.** A JSON schema with an enum of the allowed answers plus an explicit
   "unclear". Parse failures are refusals, not retries.
3. **Guard with a decoy.** Before accepting an answer, ask the same question about the same
   input with every real option replaced by options that are valid but known not to be the
   case. If the model confidently picks one of those, it is guessing on this input, and its
   answer is discarded.
4. **Never trust the confidence number.** It is a token, not a measurement. The decoy is the
   measurement.

## Testing it

The test that matters is the control, not the success case. A model that gets the right answer
on the right question has proved nothing until it declines the wrong question. Write both, and
report both:

- `ask(real options)` -> expect the true answer.
- `ask(decoy options)` -> expect "unclear".

Run them against real captured input, not only simulated input. Simulated scenes are usually
cleaner than reality, so a guard that never fires in simulation may fire constantly on the
rig - which is information, not a bug.

## When the guard fires often

That means the input is not readable, and the fix is upstream of the model:

- Crop to the region the question is about. A whole cluttered desk is a much harder problem
  than the two cells in question.
- Fix the lighting before blaming the model. Half a workspace in shadow defeats colour
  classifiers and vision models alike.
- Feed before/after pairs rather than a single frame when the question is about a change.

## Cost and latency

Keep the advisory path off the critical loop where you can: run commentary in a background
worker so a slow model never delays the machine's own decision. Log every prompt, answer and
latency; those logs are the evaluation set for the next model.

## Done when

- The deterministic path still decides everything it can.
- The model's output is schema-constrained and includes a refusal.
- A decoy control exists, runs on real input, and its outcome is visible in the logs.
- The system behaves correctly when the model is unavailable, empty, or wrong.

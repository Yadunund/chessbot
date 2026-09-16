# One line of the robot's visible reasoning, published by the brain as it works.
#
# Replaces confirmation prompts: instead of asking "did you play Nf3?", the
# robot says what it saw, what it decided and what it did.

std_msgs/Header header

uint8 PERCEIVE=0               # what was seen
uint8 DECIDE=1                 # a choice that was made
uint8 PLAN=2                   # how it intends to act
uint8 ACT=3                    # a capability was called
uint8 VERIFY=4                 # the result was checked
uint8 RECOVER=5                # handling a failure
uint8 EXPLAIN=6                # commentary (the only kind a model may write)
uint8 kind

string text

# True when a model generated `text`, so the UI can mark it as such.
bool model_generated

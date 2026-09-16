# The whole state of the system, owned and published by the brain.
#
# One state concept: `phase` says what the system is doing, including the game
# states. Everything a UI needs to render the main screen is here.

std_msgs/Header header

# What the system is doing right now.
uint8 PHASE_IDLE=0             # no game, nothing running
uint8 PHASE_CALIBRATING=1      # calibration action in progress
uint8 PHASE_SETUP=2            # waiting for the pieces to be set up / verified
uint8 PHASE_HUMAN_TURN=3       # human to move; their clock runs
uint8 PHASE_READING_BOARD=4    # clock pressed; working out what changed
uint8 PHASE_THINKING=5         # choosing the reply
uint8 PHASE_MOVING=6           # arm executing the reply
uint8 PHASE_VERIFYING=7        # checking the board after the move
uint8 PHASE_NEEDS_HELP=8       # stuck; a human has to intervene
uint8 PHASE_GAME_OVER=9
uint8 phase

# How much the system does without asking (approval gates, dev commands).
uint8 MODE_PROTOTYPE=0         # everything is approved by a human first
uint8 MODE_SUPERVISED=1        # recovery and calibration are approved
uint8 MODE_AUTONOMOUS=2        # plays on its own
uint8 mode

# Position after all committed moves, in FEN.
string fen

uint8 SIDE_WHITE=0
uint8 SIDE_BLACK=1
uint8 turn                     # side to move
uint8 robot_side               # side the robot plays

# Committed moves in UCI, oldest first. The UI renders the move list from this.
string[] moves

uint8 RESULT_IN_PROGRESS=0
uint8 RESULT_WHITE_WON=1
uint8 RESULT_BLACK_WON=2
uint8 RESULT_DRAW=3
uint8 result

# Engine strength; 0 means full strength.
int32 engine_elo

# Clocks. Remaining time at `clock_last_switch` for each side; a UI ticks the
# running side down locally instead of us republishing at 1 Hz.
int64 white_ms
int64 black_ms
uint8 CLOCK_NONE=0
uint8 CLOCK_WHITE=1
uint8 CLOCK_BLACK=2
uint8 clock_running
builtin_interfaces/Time clock_last_switch

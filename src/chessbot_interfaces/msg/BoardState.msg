# What perception currently sees on the board.
#
# Produced by the perception component in response to GetBoardState.
# It knows nothing about the rules or whose turn it is: only what is on each
# square right now, and how sure it is.

# stamp: capture time of the camera frame this was computed from, so consumers
#        can line these facts up with the image (and with what the recorder logged).
# frame_id: the camera optical frame the board was detected in.
std_msgs/Header header

uint8 EMPTY=0
uint8 WHITE=1
uint8 BLACK=2
uint8 UNKNOWN=3

# Occupancy per square, indexed file + 8 * rank, so a1 = 0, b1 = 1, a2 = 8, h8 = 63.
uint8[64] squares

# Confidence in the corresponding entry of `squares`, 0.0 to 1.0.
float32[64] confidence

# False when the board could not be located in the image, in which case every
# square is UNKNOWN.
bool board_detected

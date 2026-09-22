# The board at one moment: what perception made of it, and the pixels it saw.
#
# Either half may be absent, and both absences are normal: a detector that works on
# occupancy ignores the image, one that looks at pixels ignores the occupancy, and
# the very first observation of a game is taken before perception has learned what
# the squares look like.

# Occupancy and confidence per square. `state.board_detected` is false when the
# board could not be read, in which case every square is UNKNOWN.
BoardState state

# The camera frame the state was computed from, PNG or JPEG. Empty when none was
# captured; `format` says which encoding.
sensor_msgs/CompressedImage image

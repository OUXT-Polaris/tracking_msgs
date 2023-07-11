# A list of 2D trackings, for a multi-object 2D detector.

std_msgs/Header header

# A list of the detected proposals. A multi-proposal detector might generate
#   this list with many candidate tracking generated from a single input.
Tracking2D[] trackings

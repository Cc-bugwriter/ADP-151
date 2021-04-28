# This message defines meta information for a traffic light status
Header header

# Each bulb is assigned with an Id
# The TLR node convention is to use the yellow bulb's ID
int32 light_id

# The result as provided by the tlr nodes
# Red and Yellow states are both treated as STOP
# adjusted based on CarMaker reference Manual
# ALL lights off       = 0
# GREEN    = 1
# YELLOW    = 2
# RED  = 3
# RED-YELLOW light on  = 4
int32 recognition_result

#...............Only available in CarMaker......................
# Reaminging time until next phase, may not exits in reality
float32 reamining_time

#Position of traffic light, could be included in Map
geometry_msgs/Pose pose

#Name of the traffic light
string light_name_
#...............................................................

# This string is used by the Audio Player
string recognition_result_str

# LaneId to which this traffic light result belongs to
# this Id is defined by the ADAS MAP
int32 lane_id


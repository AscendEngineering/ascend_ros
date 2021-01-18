# mission_point.msg
#
# ROS representation of MAVLink MISSION_ITEM

uint16 number
uint16 command
bool is_current
bool autocontinue
uint16 param1
float64 x_lat
float64 y_long
float64 z_alt
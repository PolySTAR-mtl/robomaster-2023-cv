# Movement.msg
## Describes the desired velocity of the robot. Linear speed is in m/s, angular in rad/s
## Note : this does not take into account the 4 DOF offered by the robot, maybe to be improved ?

float32 v_x
float32 v_y
float32 omega

# Standard Header
Header header

# Object label
string label

# Object label ID
int16 label_id

# Object confidence level (1-99)
float32 confidence

# Object centroid
geometry_msgs/Point32 position

# Object velocity
geometry_msgs/Vector3 linear_vel

# Tracking state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
int8 tracking_state 

# 2D Bounding box projected to Camera image
#      0 ------- 1
#      |         |
#      |         |
#      |         |
#      3 ------- 2
geometry_msgs/Point32[4] bbox_2d

# 3D Bounding box in world frame
#      1 ------- 2
#     /.        /|
#    0 ------- 3 |
#    | .       | |           
#    | 5.......| 6
#    |.        |/       
#    4 ------- 7
geometry_msgs/Point32[8] bbox_3d

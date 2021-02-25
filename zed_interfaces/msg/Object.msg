# Object label
string label

# Object label ID
int16 label_id

# Object sub
string sublabel

# Object confidence level (1-99)
float32 confidence

# Object centroid position
float32[3] position

# Position covariance
float32[6] position_covariance

# Object velocity
float32[3] velocity

# Tracking available
bool tracking_available

# Tracking state
# 0 -> OFF (object not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
int8 tracking_state

# Action state
# 0 -> IDLE
# 2 -> MOVING
int8 action_state

# 2D Bounding box projected to Camera image
zed_interfaces/BoundingBox2Di bounding_box_2d

# 3D Bounding box in world frame
zed_interfaces/BoundingBox3D bounding_box_3d

# 3D dimensions (width, height, lenght)
float32[3] dimensions_3d

# Is skeleton available?
bool skeleton_available

# 2D Bounding box projected to Camera image of the person head
zed_interfaces/BoundingBox2Df head_bounding_box_2d

# 3D Bounding box in world frame of the person head
zed_interfaces/BoundingBox3D head_bounding_box_3d

# 3D position of the centroid of the person head
float32[3] head_position

# 2D Person skeleton projected to Camera image
zed_interfaces/Skeleton2D skeleton_2d

# 3D Person skeleton in world frame
zed_interfaces/Skeleton3D skeleton_3d

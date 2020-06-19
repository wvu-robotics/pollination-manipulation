# header
std_msgs/Header header

# flower id
int16 id

#number of times observed
int16 num_obs

#accum. probability of detection
float32 prob

# flower pose
geometry_msgs/PointStamped point
geometry_msgs/Vector3Stamped vec

# Delete this later
geometry_msgs/Pose pose

# Eigen Decomp of PointCloud
manipulation_common/EigenDecomp eigen_decomp

# Pointcloud
sensor_msgs/PointCloud2 point_cloud

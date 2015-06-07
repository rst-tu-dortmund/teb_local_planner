# Message that contains a list of polygon shaped obstacles.
# Special types:
# Polygon with 1 vertex: Point obstacle
# Polygon with 2 vertices: Line obstacle
# Polygon with more than 2 vertices: First and last points are assumed to be connected
#
# If optional properties (orientaions or velocities) are provided,
# each container size must match the number of obstacles
# otherwise let them empty.


std_msgs/Header header

# Actual obstacle positions (polygon descriptions)
geometry_msgs/PolygonStamped[] obstacles

# Obstacle IDs [optional]
# Specify IDs in order to provide (temporal) relationships
# between obstacles among multiple messages.
# If provided it must be size(ids) = size(obstacles)
uint32[] ids

# Individual orientations (centroid) [optional]
# If provided it must be size(orientations) = size(obstacles) 
geometry_msgs/QuaternionStamped[] orientations

# Individual velocities (centroid) [optional]
# If provided it must be size(velocities) = size(obstacles)
geometry_msgs/TwistWithCovariance[] velocities


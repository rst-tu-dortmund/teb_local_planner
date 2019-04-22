#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32


def publish_obstacle_msg():
  pub = rospy.Publisher('/custom_obstacles', ObstacleArrayMsg, queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("test_obstacle_msg")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
  
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 0
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = 1.5
  obstacle_msg.obstacles[0].polygon.points[0].y = 0
  obstacle_msg.obstacles[0].polygon.points[0].z = 0


  # Add line obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 1
  line_start = Point32()
  line_start.x = -2.5
  line_start.y = 0.5
  #line_start.y = -3
  line_end = Point32()
  line_end.x = -2.5
  line_end.y = 2
  #line_end.y = -4
  obstacle_msg.obstacles[1].polygon.points = [line_start, line_end]
  
  # Add polygon obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 2
  v1 = Point32()
  v1.x = -1
  v1.y = -1
  v2 = Point32()
  v2.x = -0.5
  v2.y = -1.5
  v3 = Point32()
  v3.x = 0
  v3.y = -1
  obstacle_msg.obstacles[2].polygon.points = [v1, v2, v3]

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 3
  v1 = Point32()
  v1.x = 6
  v1.y = 8.8
  v2 = Point32()
  v2.x = 6
  v2.y = 9.45
  v3 = Point32()
  v3.x = 8.5
  v3.y = 9.45
  v4 = Point32()
  v4.x = 8.5
  v4.y = 8.8
  obstacle_msg.obstacles[3].polygon.points = [v1, v2, v3, v4]
  
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 4
  v1 = Point32()
  v1.x = 9.45
  v1.y = 8.8
  v2 = Point32()
  v2.x = 9.45
  v2.y = 9.45
  v3 = Point32()
  v3.x = 12
  v3.y = 9.45
  v4 = Point32()
  v4.x = 12
  v4.y = 8.8
  obstacle_msg.obstacles[4].polygon.points = [v1, v2, v3, v4]

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 5
  v1 = Point32()
  v1.x = 6.5
  v1.y = 6.65
  v2 = Point32()
  v2.x = 6.5
  v2.y = 7.35
  v3 = Point32()
  v3.x = 13.95
  v3.y = 7.35
  v4 = Point32()
  v4.x = 13.95
  v4.y = 6.65
  obstacle_msg.obstacles[5].polygon.points = [v1, v2, v3, v4]

  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[1].id = 6
  v1 = Point32()
  v1.x = 6.5
  v1.y = 5.05
  v2 = Point32()
  v2.x = 6.5
  v2.y = 5.7
  v3 = Point32()
  v3.x = 13.9
  v3.y = 5.7
  v4 = Point32()
  v4.x = 13.9
  v4.y = 5.05
  obstacle_msg.obstacles[6].polygon.points = [v1, v2, v3, v4]
  

  r = rospy.Rate(10) # 10hz
  # t = 0.0
  while not rospy.is_shutdown():
    
    # Vary y component of the point obstacle
    # obstacle_msg.obstacles[0].polygon.points[0].y = 1*math.sin(t)
    # t = t + 0.1
    
    pub.publish(obstacle_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass


#!/usr/bin/env python

# Author: franz.albers@tu-dortmund.de

import rospy, math, tf
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32, QuaternionStamped, Quaternion, TwistWithCovariance
from tf.transformations import quaternion_from_euler


def publish_obstacle_msg():
  pub = rospy.Publisher('/test_optim_node/obstacles', ObstacleArrayMsg, queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  rospy.init_node("test_obstacle_msg")

  y_0 = -3.0
  vel_x = 0.0
  vel_y = 0.3
  range_y = 6.0

  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map
  
  # Add point obstacle
  obstacle_msg.obstacles.append(ObstacleMsg())
  obstacle_msg.obstacles[0].id = 99
  obstacle_msg.obstacles[0].polygon.points = [Point32()]
  obstacle_msg.obstacles[0].polygon.points[0].x = -1.5
  obstacle_msg.obstacles[0].polygon.points[0].y = 0
  obstacle_msg.obstacles[0].polygon.points[0].z = 0

  yaw = math.atan2(vel_y, vel_x)
  q = tf.transformations.quaternion_from_euler(0,0,yaw)
  obstacle_msg.obstacles[0].orientation = Quaternion(*q)

  obstacle_msg.obstacles[0].velocities.twist.linear.x = vel_x
  obstacle_msg.obstacles[0].velocities.twist.linear.y = vel_y
  obstacle_msg.obstacles[0].velocities.twist.linear.z = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.x = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.y = 0
  obstacle_msg.obstacles[0].velocities.twist.angular.z = 0

  r = rospy.Rate(10) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
    
    # Vary y component of the point obstacle
    if (vel_y >= 0):
      obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + (vel_y*t)%range_y
    else:
      obstacle_msg.obstacles[0].polygon.points[0].y = y_0 + (vel_y*t)%range_y - range_y

    t = t + 0.1
    
    pub.publish(obstacle_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass


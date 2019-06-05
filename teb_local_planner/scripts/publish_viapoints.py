#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def publish_via_points_msg():
  pub = rospy.Publisher('/test_optim_node/via_points', Path, queue_size=1)
  rospy.init_node("test_via_points_msg")


  via_points_msg = Path() 
  via_points_msg.header.stamp = rospy.Time.now()
  via_points_msg.header.frame_id = "odom" # CHANGE HERE: odom/map
  
  # Add via-points
  point1 = PoseStamped()
  point1.pose.position.x = 0.0;
  point1.pose.position.y = 1.5;

  point2 = PoseStamped()
  point2.pose.position.x = 2.0;
  point2.pose.position.y = -0.5;


  via_points_msg.poses = [point1, point2]

  r = rospy.Rate(5) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
        
    pub.publish(via_points_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_via_points_msg()
  except rospy.ROSInterruptException:
    pass


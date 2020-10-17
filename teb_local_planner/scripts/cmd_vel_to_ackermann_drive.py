#!/usr/bin/env python3

# Author: christoph.roesmann@tu-dortmund.de

import rclpy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global cmd_angle_instead_rotvel
  
  v = data.linear.x
  # if cmd_angle_instead_rotvel is true, the rotational velocity is already converted in the C++ node
  # in this case this script only needs to do the msg conversion from twist to Ackermann drive
  if cmd_angle_instead_rotvel:
    steering = data.angular.z
  else:
    steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = AckermannDriveStamped()
  msg.header.stamp = node.get_clock().now().to_msg()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = float(steering)
  msg.drive.speed = float(v)
  
  pub.publish(msg)
  




if __name__ == '__main__': 
  rclpy.init()
  global node
  node = rclpy.create_node('cmd_vel_to_ackermann_drive')
  
  twist_cmd_topic = node.declare_parameter("twist_cmd_topic", "/cmd_vel").value
  ackermann_cmd_topic = node.declare_parameter("ackermann_cmd_topic", "/ackermann_cmd").value
  wheelbase = node.declare_parameter("wheelbase", 1.0).value
  frame_id = node.declare_parameter('frame_id', 'odom').value
  cmd_angle_instead_rotvel = node.declare_parameter('cmd_angle_instead_rotvel', False).value

  node.create_subscription(Twist, twist_cmd_topic, cmd_callback, 1)
  pub = node.create_publisher(AckermannDriveStamped, ackermann_cmd_topic, 1)
  
  rclpy.spin(node)


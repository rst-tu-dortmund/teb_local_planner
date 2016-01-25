#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and exports data to a mat file.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy.io as sio
import time

def feedback_callback(data):
  global got_data

  if not data.trajectories: # empty
    trajectory = []
    return
  
  if got_data:		
    return

  got_data = True
  
  # copy trajectory
  trajectories = []
  for traj in data.trajectories:
    trajectory = []
#    # store as struct and cell array
#    for point in traj.trajectory:
#      (roll,pitch,yaw) = euler_from_quaternion([point.pose.orientation.x,point.pose.orientation.y,point.pose.orientation.z,point.pose.orientation.w])
#      pose = {'x': point.pose.position.x, 'y': point.pose.position.y, 'theta': yaw}
#      velocity = {'v': point.velocity.linear.x, 'omega': point.velocity.angular.z}
#      time_from_start = point.time_from_start.to_sec()
#      trajectory.append({'pose': pose, 'velocity': velocity, 'time_from_start': time_from_start})
      
    # store as all-in-one mat
    arr = np.zeros([6, len(traj.trajectory)], dtype='double'); # x, y, theta, v, omega, t
    for index, point in enumerate(traj.trajectory):
      arr[0,index] = point.pose.position.x
      arr[1,index] = point.pose.position.y
      (roll,pitch,yaw) = euler_from_quaternion([point.pose.orientation.x,point.pose.orientation.y,point.pose.orientation.z,point.pose.orientation.w])
      arr[2,index] = yaw
      arr[3,index] = point.velocity.linear.x
      arr[4,index] = point.velocity.angular.z
      arr[5,index] = point.time_from_start.to_sec()
      
#   trajectories.append({'raw': trajectory, 'mat': arr})
    trajectories.append({'data': arr, 'legend': ['x','y','theta','v','omega','t']})
  
  # copy obstacles
  obstacles = []
  for obst in data.obstacles:
    #polygon = []
    #for point in obst.polygon.points:
    #  polygon.append({'x': point.x, 'y': point.y, 'z': point.z})
    obst_arr = np.zeros([2, len(obst.polygon.points)], dtype='double'); # x, y
    for index, point in enumerate(obst.polygon.points):
      obst_arr[0, index] = point.x
      obst_arr[1, index] = point.y
    #obstacles.append(polygon)
    obstacles.append({'data': obst_arr, 'legend': ['x','y']})
  
  
  # create main struct:
  mat = {'selected_trajectory_idx': data.selected_trajectory_idx, 'trajectories': trajectories, 'obstacles': obstacles}

  timestr = time.strftime("%Y%m%d_%H%M%S")
  filename = "teb_data_" + timestr + '.mat'
  
  rospy.loginfo("Saving mat-file '%s'.", filename)
  sio.savemat(filename, mat)
  

  
  
  
def feedback_exporter():
  global got_data

  rospy.init_node("export_to_mat", anonymous=True)
  
  
  topic_name = "/test_optim_node/teb_feedback" # define feedback topic here!

  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) 

  rospy.loginfo("Waiting for feedback message on topic %s.", topic_name)
 
  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    if got_data:
      rospy.loginfo("Data export completed.")
      return

    r.sleep()

if __name__ == '__main__': 
  try:
    global got_data
    got_data = False
    feedback_exporter()
  except rospy.ROSInterruptException:
    pass


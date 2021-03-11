#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32
import numpy as np
import matplotlib.pyplot as plotter

def feedback_callback(data):
  global trajectory

  if not data.trajectories: # empty
    trajectory = []
    return
  trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
  
  
def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()

  
  
def velocity_plotter():
  global trajectory
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  
  topic_name = "/test_optim_node/teb_feedback"
  topic_name = rospy.get_param('~feedback_topic', topic_name)
  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) # define feedback topic here!

  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # two subplots sharing the same t axis
  fig, (ax_v, ax_omega) = plotter.subplots(2, sharex=True)
  plotter.ion()
  plotter.show()
  

  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():
    
    t = []
    v = []
    omega = []
    
    for point in trajectory:
      t.append(point.time_from_start.to_sec())
      v.append(point.velocity.linear.x)
      omega.append(point.velocity.angular.z)
          
    plot_velocity_profile(fig, ax_v, ax_omega, np.asarray(t), np.asarray(v), np.asarray(omega))
        
    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass


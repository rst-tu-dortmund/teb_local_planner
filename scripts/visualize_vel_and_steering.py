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


def plot_velocity_profile(fig, ax_v, ax_omega, ax_steering, ax_rate, t, v, s, omega, steering, total_steering, av_steerrate, sum_steerrate):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  #ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  ax_steering.cla()
  ax_steering.grid()
  ax_steering.set_ylabel('Steering angle [rad]')
  ax_steering.plot(t, steering, '-bx')
  ax_steering.set_title('Total steering: {}'.format(total_steering))
  ax_rate.cla() #Plot steering rate against time
  ax_rate.grid()
  ax_rate.set_ylabel("Steering rate")
  ax_rate.plot(t,s, '-bx')
  ax_rate.set_xlabel('Time [s]')
  ax_rate.set_title("Average steering rate: {}, sum of steering rate: {}".format(av_steerrate, sum_steerrate))
  fig.canvas.draw()





def velocity_plotter():
  global trajectory
  rospy.init_node("visualize_vel_and_steering", anonymous=True)

  topic_name = "/test_optim_node/teb_feedback"
  topic_name = rospy.get_param('~feedback_topic', topic_name)
  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) # define feedback topic here!

  wheelbase = 1.0
  wheelbase = rospy.get_param('~wheelbase', wheelbase)

  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  # four subplots sharing the same t axis
  fig, (ax_v, ax_omega, ax_steering, ax_rate) = plotter.subplots(4, sharex=True)
  plotter.ion()
  plotter.show()


  r = rospy.Rate(2) # define rate here
  while not rospy.is_shutdown():

    t = []
    v = []
    omega = []
    steering = []

    for point in trajectory:
      t.append(point.time_from_start.to_sec())
      v.append(point.velocity.linear.x)
      omega.append(point.velocity.angular.z)
      if point.velocity.linear.x == 0:
        steering.append( 0.0 )
      else:
        steering.append( math.atan( wheelbase / point.velocity.linear.x * point.velocity.angular.z )  )
    
    #Steering rate
    s =[]
    for i in range(np.size(np.asarray(steering))-1):
      s.append((steering[i+1]-steering[i])/(t[i+1]-t[i]))
    s.append(0)
    
    #Steering rate approximated by central differences
    #s =[]
    #s.append(0)
    #for i in range(np.size(np.asarray(steering))-2):
    #  s.append((steering[i+2]-steering[i])/(2*(-t[i]+t[i+2])))
    0s.append(0)


    sum_steerrate=np.sum(np.absolute(np.asarray(s))) 
    av_steerrate=sum_steerrate/np.size(np.asarray(s)) #Average steering rate

    total_steering=np.sum(np.absolute(np.asarray(steering))) #Sum of steering angles

    plot_velocity_profile(fig, ax_v, ax_omega, ax_steering, ax_rate, np.asarray(t), np.asarray(v), np.asarray(s), np.asarray(omega), np.asarray(steering), total_steering, av_steerrate, sum_steerrate)
     
    

    r.sleep()

if __name__ == '__main__': 
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass



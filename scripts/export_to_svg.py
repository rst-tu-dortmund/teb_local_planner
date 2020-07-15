#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
========================================================================================
# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and converts the current scene to a svg-image
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

It is recommendable to start this node after initialization of TEB is completed.

Requirements:
svgwrite: A Python library to create SVG drawings. http://pypi.python.org/pypi/svgwrite
=======================================================================================
"""
import roslib;
import rospy
import svgwrite
import math
import sys
import time
import random
from svgwrite import cm, mm
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32, Quaternion


# ================= PARAMETERS ==================
# TODO: In case of a more general node, change parameter to ros-parameter
# Drawing parameters: 
SCALE = 200	 # Overall scaling: 100 pixel = 1 m
MIN_POSE_DISTANCE = 0.3 # Distance between two consecutive poses in SVG-image
SCALE_VELOCITY_VEC = 0.4 # Scaling of velocity vectors -> 1 cell = 1/SCALE_VELOCITY_VEC m/s
GRID_X_MIN = -2  # Define, how many cells your grid should contain in each direction.
GRID_X_MAX = 2
GRID_Y_MIN = -2
GRID_Y_MAX = 1

# TEB parameters:
OBSTACLE_DIST = 50 *SCALE/100 # cm


# ================= FUNCTIONS ===================

def sign(number):
    """
    Signum function: get sign of a number

    @param number: get sign of this number
    @type  number: numeric type (eg. integer)
    @return:  sign of number
    @rtype:   integer {1, -1, 0}  
    """
    return cmp(number,0)

def arrowMarker(color='green', orientation='auto'):
    """
    Create an arrow marker with svgwrite

    @return:  arrow marker
    @rtype:   svg_write marker object 
    """
    arrow = svg.marker(insert=(1,5), size=(4,3), orient=orientation)
    arrow.viewbox(width=10, height=10)
    arrow.add(svg.polyline([(0,0),(10,5),(0,10),(1,5)], fill=color, opacity=1.0))
    svg.defs.add(arrow)
    return arrow

def quaternion2YawDegree(orientation):
    """
    Get yaw angle [degree] from quaternion representation

    @param orientation:	orientation in quaternions to read from
    @type  orientation:	geometry_msgs/Quaternion
    @return: yaw angle [degree]
    @rtype:  float  
    """
    yawRad = math.atan2(2*(orientation.x*orientation.y+orientation.z*orientation.w),1-2*(pow(orientation.y,2)+pow(orientation.z,2)))
    return yawRad*180/math.pi


def feedback_callback(data):
    """
    Callback for receiving TEB and obstacle information

    @param data: Received feedback message
    @type  data: visualization_msgs/Marker

    @globalparam tebList: Received TEB List
    @globaltype  tebList: teb_local_planner/FeedbackMsg
    """
    # TODO: Remove global variables
    global feedbackMsg
    
    if not feedbackMsg:
      feedbackMsg = data
      rospy.loginfo("TEB feedback message received...") 
               
   
# ================ MAIN FUNCTION ================

if __name__ == '__main__':
    rospy.init_node('export_to_svg', anonymous=True)
    
    topic_name = "/test_optim_node/teb_feedback" # define feedback topic here!

    rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) 

    rospy.loginfo("Waiting for feedback message on topic %s.", topic_name)

    rate = rospy.Rate(10.0)
    feedbackMsg = []

    timestr = time.strftime("%Y%m%d_%H%M%S")
    filename_string = "teb_svg_" + timestr + '.svg'
  
    rospy.loginfo("SVG will be written to '%s'.", filename_string)
    
    random.seed(0)

    svg=svgwrite.Drawing(filename=filename_string, debug=True)
    
    # Create viewbox -> this box defines the size of the visible drawing
    svg.viewbox(GRID_X_MIN*SCALE-1*SCALE,GRID_Y_MIN*SCALE-1*SCALE,GRID_X_MAX*SCALE-GRID_X_MIN*SCALE+2*SCALE,GRID_Y_MAX*SCALE-GRID_Y_MIN*SCALE+2*SCALE)

    # Draw grid:
    hLines = svg.add(svg.g(id='hLines', stroke='black'))
    hLines.add(svg.line(start=(GRID_X_MIN*SCALE, 0), end=(GRID_X_MAX*SCALE, 0)))
    for y in range(GRID_Y_MAX):
        hLines.add(svg.line(start=(GRID_X_MIN*SCALE, SCALE+y*SCALE), end=(GRID_X_MAX*SCALE, SCALE+y*SCALE)))
    for y in range(-GRID_Y_MIN):
        hLines.add(svg.line(start=(GRID_X_MIN*SCALE, -SCALE-y*SCALE), end=(GRID_X_MAX*SCALE, -SCALE-y*SCALE)))
    vLines = svg.add(svg.g(id='vline', stroke='black'))
    vLines.add(svg.line(start=(0, GRID_Y_MIN*SCALE), end=(0, GRID_Y_MAX*SCALE)))
    for x in range(GRID_X_MAX):
        vLines.add(svg.line(start=(SCALE+x*SCALE, GRID_Y_MIN*SCALE), end=(SCALE+x*SCALE, GRID_Y_MAX*SCALE)))
    for x in range(-GRID_X_MIN):
        vLines.add(svg.line(start=(-SCALE-x*SCALE, GRID_Y_MIN*SCALE), end=(-SCALE-x*SCALE, GRID_Y_MAX*SCALE)))


    # Draw legend:
    legend = svg.g(id='legend', font_size=25)
    stringGeometry = "Geometry: 1 Unit = 1.0m"
    legendGeometry = svg.text(stringGeometry)
    legend.add(legendGeometry)
    legend.translate(tx=GRID_X_MIN*SCALE, ty=GRID_Y_MAX*SCALE + 30) # Move legend to buttom left corner
    svg.add(legend)


    #arrow = arrowMarker() # Init arrow marker

    rospy.loginfo("Initialization completed.\nWaiting for feedback message...") 

    # -------------------- WAIT FOR CALLBACKS --------------------------  
    while not rospy.is_shutdown():
        if feedbackMsg:
            break # Leave loop after receiving all necessary TEB information (see callbacks) to finish drawing
        rate.sleep()
    # ------------------------------------------------------------------
    
    if not feedbackMsg.trajectories:
      rospy.loginfo("Received message does not contain trajectories. Shutting down...")
      sys.exit()
    
    if len(feedbackMsg.trajectories[0].trajectory) < 2:
      rospy.loginfo("Received message does not contain trajectories with at least two states (start and goal). Shutting down...")
      sys.exit()
        
    # iterate trajectories
    for index, traj in enumerate(feedbackMsg.trajectories):
      
        #color
        traj_color = svgwrite.rgb(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255), 'RGB')
   
        # Iterate through TEB positions -> Draw Paths
        points = []
        for point in traj.trajectory:
            points.append( (point.pose.position.x*SCALE,-point.pose.position.y*SCALE) ) # y is negative in image coordinates
            # svgwrite rotates clockwise!
            

        if index == feedbackMsg.selected_trajectory_idx: # highlight currently selected teb
          line = svg.add( svg.polyline(points=points, fill='none', stroke=traj_color, stroke_width=10, stroke_linecap='round', \
                          stroke_linejoin='round', opacity=1.0 ) )
        else:
          line = svg.add( svg.polyline(points=points, fill='none', stroke=traj_color, stroke_width=10, stroke_linecap='butt', \
                          stroke_linejoin='round', stroke_dasharray='10,3', opacity=1.0 ) )
        #marker_points = points[::7]
        #markerline = svg.add( svg.polyline(points=marker_points, fill='none', stroke=traj_color, stroke_width=10, opacity=0.0 ) )
        #arrow = arrowMarker(traj_color)
        #markerline.set_markers( (arrow, arrow, arrow) )
        #line.set_markers( (arrow, arrow, arrow) )
        #line['marker-start'] = arrow.get_funciri()
    
  
    # Add Start and Goal Point
    start_pose = feedbackMsg.trajectories[0].trajectory[0].pose
    goal_pose = feedbackMsg.trajectories[0].trajectory[len(feedbackMsg.trajectories[0].trajectory)-1].pose
    start_position = start_pose.position
    goal_position = goal_pose.position
    svg.add(svg.circle(center=(start_position.x*SCALE,-start_position.y*SCALE), r=10, stroke_width=1, stroke='blue', fill ='blue'))
    svg.add(svg.text("Start", (start_position.x*SCALE-70, -start_position.y*SCALE+45), font_size=35)) # Add label
    svg.add(svg.circle(center=(goal_position.x*SCALE,-goal_position.y*SCALE), r=10, stroke_width=1, stroke='red', fill ='red'))
    svg.add(svg.text("Goal", (goal_position.x*SCALE-40, -goal_position.y*SCALE+45), font_size=35)) # Add label
    
    # draw start arrow
    start_arrow = svg.polyline([(0,-1),(6,-1),(5,-5),(15,0),(5,5),(6,1),(0,1)], fill='blue', opacity=1.0)
    start_arrow.translate(start_position.x*SCALE,-start_position.y*SCALE)
    start_arrow.rotate( quaternion2YawDegree(start_pose.orientation)  )
    start_arrow.scale(3)
    svg.add(start_arrow)
    
    # draw goal arrow
    goal_arrow = svg.polyline([(0,-1),(6,-1),(5,-5),(15,0),(5,5),(6,1),(0,1)], fill='red', opacity=1.0)
    goal_arrow.translate(goal_position.x*SCALE,-goal_position.y*SCALE)
    goal_arrow.rotate( quaternion2YawDegree(goal_pose.orientation)  )
    goal_arrow.scale(3)
    svg.add(goal_arrow)
    
    # Draw obstacles
    for obstacle in feedbackMsg.obstacles:
      if len(obstacle.polygon.points) == 1: # point obstacle
          point = obstacle.polygon.points[0]
          svg.add(svg.circle(center=(point.x*SCALE,-point.y*SCALE), r=OBSTACLE_DIST, stroke_width=1, stroke='grey', fill ='grey', opacity=0.3))
          svg.add(svg.circle(center=(point.x*SCALE,-point.y*SCALE), r=15, stroke_width=1, stroke='black', fill ='black'))
          svg.add(svg.text("Obstacle", (point.x*SCALE-70, -point.y*SCALE+45), font_size=35)) # Add label
      if len(obstacle.polygon.points) == 2: # line obstacle
          line_start = obstacle.polygon.points[0]
          line_end = obstacle.polygon.points[1]
          svg.add(svg.line(start=(line_start.x*SCALE,-line_start.y*SCALE), end=(line_end.x*SCALE,-line_end.y*SCALE), stroke='black', fill='gray', stroke_width=1, opacity=1.0))
          svg.add(svg.text("Obstacle", (line_start.x*SCALE-70, -line_start.y*SCALE+45), font_size=35)) # Add label
      if len(obstacle.polygon.points) > 2: # polygon obstacle
          vertices = []
          for point in obstacle.polygon.points:
            vertices.append((point.x*SCALE, -point.y*SCALE))
          svg.add(svg.polygon(points=vertices, stroke='black', fill='gray', stroke_width=1, opacity=1.0))
          svg.add(svg.text("Obstacle", (obstacle.polygon.points[0].x*SCALE-70, -obstacle.polygon.points.y*SCALE+45), font_size=35)) # Add label
          
          
    
    # Save svg to file (svg_output.svg) and exit node            
    svg.save() 
    
    rospy.loginfo("Drawing completed.") 

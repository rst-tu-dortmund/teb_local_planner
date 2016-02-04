/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/


#ifndef ROBOT_SHAPE_MODEL_H
#define ROBOT_SHAPE_MODEL_H

#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/obstacles.h>
#include <visualization_msgs/Marker.h>

namespace teb_local_planner
{

/**
 * @class BaseRobotShapeModel
 * @brief Abstract class that defines the interface for robot shape models
 * 
 * The robot model class is currently used in optimization only, since
 * taking the navigation stack footprint into account might be
 * inefficient. The footprint is only used for checking feasibility.
 */
class BaseRobotShapeModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    */
  BaseRobotShapeModel()
  {
  }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~BaseRobotShapeModel()
  {
  }


  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const = 0;

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information.
    * The header, namespace and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] marker marker message to be filled
    * @return \c true, if the robot can be visualized and the message is filled accordingly.
    */
  virtual bool visualizeRobot(const PoseSE2& current_pose, visualization_msgs::Marker& marker ) const {return false;}

	

public:	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//! Abbrev. for shared obstacle pointers
typedef boost::shared_ptr<BaseRobotShapeModel> RobotShapeModelPtr;
//! Abbrev. for shared obstacle const pointers
typedef boost::shared_ptr<const BaseRobotShapeModel> RobotShapeModelConstPtr;



/**
 * @class PointRobotShape
 * @brief Class that defines a point-robot
 * 
 * Instead of using a CircularRobotModel this class might
 * be utitilzed and the robot radius can be added to the mininum distance 
 * parameter. This avoids a subtraction of zero each time a distance is calculated.
 */
class PointRobotShape : public BaseRobotShapeModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    */
  PointRobotShape() {}
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~PointRobotShape() {}

  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    return obstacle->getMinimumDistance(current_pose.position());
  }

};


/**
 * @class CircularRobotShape
 * @brief Class that defines the a robot of circular shape
 */
class CircularRobotShape : public BaseRobotShapeModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param radius radius of the robot
    */
  CircularRobotShape(double radius) : radius_(radius) { }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~CircularRobotShape() { }

  /**
    * @brief Set radius of the circular robot
    * @param radius radius of the robot
    */
  void setRadius(double radius) {radius_ = radius;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    return obstacle->getMinimumDistance(current_pose.position()) - radius_;
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information.
    * The header, namespace and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] marker marker message to be filled
    * @return \c true, if the robot can be visualized and the message is filled accordingly.
    */
  virtual bool visualizeRobot(const PoseSE2& current_pose, visualization_msgs::Marker& marker ) const 
  {
    marker.type = visualization_msgs::Marker::CYLINDER;
    current_pose.toPoseMsg(marker.pose);
    marker.scale.x = marker.scale.y = 2*radius_; // scale = diameter
    marker.scale.z = 0.05;
    marker.color.a = 0.5;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    return true;
  }

private:
    
  double radius_;
};


/**
 * @class TwoCirclesRobotShape
 * @brief Class that approximates the robot with two shifted circles
 */
class TwoCirclesRobotShape : public BaseRobotShapeModel
{
public:
  
  /**
    * @brief Default constructor of the abstract obstacle class
    * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
    * @param front_radius radius of the front circle
    * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
    * @param rear_radius radius of the front circle
    */
  TwoCirclesRobotShape(double front_offset, double front_radius, double rear_offset, double rear_radius) 
    : front_offset_(front_offset), front_radius_(front_radius), rear_offset_(rear_offset), rear_radius_(rear_radius) { }
  
  /**
   * @brief Virtual destructor.
   */
  virtual ~TwoCirclesRobotShape() { }

  /**
   * @brief Set parameters of the shape
   * @param front_offset shift the center of the front circle along the robot orientation starting from the center at the rear axis (in meters)
   * @param front_radius radius of the front circle
   * @param rear_offset shift the center of the rear circle along the opposite robot orientation starting from the center at the rear axis (in meters)
   * @param rear_radius radius of the front circle
   */
  void setParameters(double front_offset, double front_radius, double rear_offset, double rear_radius) 
  {front_offset_=front_offset; front_radius_=front_radius; rear_offset_=rear_offset; rear_radius_=rear_radius;}
  
  /**
    * @brief Calculate the distance between the robot and an obstacle
    * @param current_pose Current robot pose
    * @param obstacle Pointer to the obstacle
    * @return Euclidean distance to the robot
    */
  virtual double calculateDistance(const PoseSE2& current_pose, const Obstacle* obstacle) const
  {
    Eigen::Vector2d dir = current_pose.orientationUnitVec();
    double dist_front = obstacle->getMinimumDistance(current_pose.position() + front_offset_*dir) - front_radius_;
    double dist_rear = obstacle->getMinimumDistance(current_pose.position() - rear_offset_*dir) - rear_radius_;
    return std::min(dist_front, dist_rear);
  }

  /**
    * @brief Visualize the robot using a markers
    * 
    * Fill a marker message with all necessary information.
    * The header, namespace and marker lifetime will be overwritten.
    * @param current_pose Current robot pose
    * @param[out] marker marker message to be filled
    * @return \c true, if the robot can be visualized and the message is filled accordingly.
    */
  virtual bool visualizeRobot(const PoseSE2& current_pose, visualization_msgs::Marker& marker ) const 
  {
    return false;
  }

private:
    
  double front_offset_;
  double front_radius_;
  double rear_offset_;
  double rear_radius_;
  
};






} // namespace teb_local_planner

#endif /* ROBOT_MODEL_H */

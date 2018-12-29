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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef EDGE_JERK_H_
#define EDGE_JERK_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{

/**
 * @class EdgeJerk
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 *
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \mathbf{s}_{ip3}, \Delta T_i, \Delta T_{ip1}, \Delta T_{ip2} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [j, \ddot{omega}]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (three times) and the position parts of all three poses \n
 * \e \ddot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkStart
 * @see EdgeJerkGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkStart() and EdgeJerkGoal() for defining boundary values!
 */
class EdgeJerk : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerk()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeJerkStart
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j, \ddot{omega}]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (three times) and the position parts of the poses. \n
 * \e \ddot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerk
 * @see EdgeJerkGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkGoal() for defining boundary values at the end of the trajectory!
 */
class EdgeJerkStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkStart()
  {
    _measurement = NULL;
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the jerk
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkGoal
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, a goal velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j, \ddot{omega}]^T ) \cdot weight \f$. \n
 * \e j is calculated using the difference quotient (three times) and the position parts of the poses \n
 * \e \ddot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational jerk and
 * the second one the rotational jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerk
 * @see EdgeJerkStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkStart() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeJerkGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkGoal()
  {
    _measurement = NULL;
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the jerk
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic1
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \mathbf{s}_{ip3}, \Delta T_i, \Delta T_{ip1}, \Delta T_{ip2} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [j]^T ) \cdot weight \f$. \n
 * \e j denotes the normalized jerk (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 1: a represents the normalized jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic1Start
 * @see EdgeJerkHolonomic1Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic1Start() and EdgeJerkHolonomic1Goal() for defining boundary values!
 */
class EdgeJerkHolonomic1 : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic1()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeJerkHolonomic1Start
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j]^T ) \cdot weight \f$. \n
 * \e j denotes the normalized jerk (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 1: a represents the normalized jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic1
 * @see EdgeJerkHolonomic1Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic1Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeJerkHolonomic1Start : public BaseTebMultiEdge<1, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic1Start()
  {
    this->resize(5);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the jerk
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic1Goal
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, a goal velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j]^T ) \cdot weight \f$. \n
 * \e j denotes the normalized jerk (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 1: a represents the normalized jerk.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic1
 * @see EdgeJerkHolonomic1Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic1Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeJerkHolonomic1Goal : public BaseTebMultiEdge<1, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic1Goal()
  {
    _measurement = NULL;
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the jerk
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic3
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 *
 *       x              x              x              x
 *       --          /                                    \
 * y   |----|     y   |----|     y / |----| \   y   |----|
 *     |    |         |    | |       |    |       | |    |
 *   \ |----| /       |----|         |----|         |----|
 *                   \                 --                 /
 * This is for holonomic type 3: 3 wheels, 0 degrees
 *                       type 4: 3 wheels, 30 degrees
 *                       type 5: 3 wheels, 60 degrees
 *                       type 6: 3 wheels, 90 degrees
 *
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \mathbf{s}_{ip3}, \Delta T_i, \Delta T_{ip1}, \Delta T_{ip2} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w3} denote the normalized jerk of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: they represent the normalized jerk of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic3Start
 * @see EdgeJerkHolonomic3Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic3Start() and EdgeJerkHolonomic3Goal() for defining boundary values!
 */
class EdgeJerkHolonomic3 : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic3()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeJerkHolonomic3Start
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 *
 *       x              x              x              x
 *       --          /                                    \
 * y   |----|     y   |----|     y / |----| \   y   |----|
 *     |    |         |    | |       |    |       | |    |
 *   \ |----| /       |----|         |----|         |----|
 *                   \                 --                 /
 * This is for holonomic type 3: 3 wheels, 0 degrees
 *                       type 4: 3 wheels, 30 degrees
 *                       type 5: 3 wheels, 60 degrees
 *                       type 6: 3 wheels, 90 degrees
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w3} denote the normalized jerk of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: they represent the normalized jerk of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic3
 * @see EdgeJerkHolonomic3Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic3Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeJerkHolonomic3Start : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic3Start()
  {
    this->resize(5);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the jerk
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic3Goal
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 *
 *       x              x              x              x
 *       --          /                                    \
 * y   |----|     y   |----|     y / |----| \   y   |----|
 *     |    |         |    | |       |    |       | |    |
 *   \ |----| /       |----|         |----|         |----|
 *                   \                 --                 /
 * This is for holonomic type 3: 3 wheels, 0 degrees
 *                       type 4: 3 wheels, 30 degrees
 *                       type 5: 3 wheels, 60 degrees
 *                       type 6: 3 wheels, 90 degrees
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, a goal velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w3} denote the normalized jerk of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: they represent the normalized jerk of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic3
 * @see EdgeJerkHolonomic3Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic3Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeJerkHolonomic3Goal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic3Goal()
  {
    _measurement = NULL;
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the jerk
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic4
 * @brief Edge defining the cost function for limiting the translational and rotational jerk.
 *
 *       x              x
 *    /      \          --
 * y   |----|     y   |----|
 *     |    |       | |    | |
 *     |----|         |----|
 *    \      /          --
 * This is for holonomic type 1: 4 wheels, 45 degrees
 *                       type 2: 4 wheels, 0 degrees
 *
 * The edge depends on seven vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \mathbf{s}_{ip3}, \Delta T_i, \Delta T_{ip1}, \Delta T_{ip2} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}, j_{w4}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w4} denote the normalized jerk of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 4: they represent the normalized jerk of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic4Start
 * @see EdgeJerkHolonomic4Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic4Start() and EdgeJerkHolonomic4Goal() for defining boundary values!
 */
class EdgeJerkHolonomic4 : public BaseTebMultiEdge<4, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic4()
  {
    this->resize(7);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeJerkHolonomic4Start
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the beginning of the trajectory.
 *
 *       x              x
 *    /      \          --
 * y   |----|     y   |----|
 *     |    |       | |    | |
 *     |----|         |----|
 *    \      /          --
 * This is for holonomic type 1: 4 wheels, 45 degrees
 *                       type 2: 4 wheels, 0 degrees
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}, j_{w4}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w4} denote the normalized jerk of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 4: they represent the normalized jerk of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic4
 * @see EdgeJerkHolonomic4Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic4Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeJerkHolonomic4Start : public BaseTebMultiEdge<4, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic4Start()
  {
    this->resize(5);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the jerk
   * @param vel_start twist message containing the translational and rotational velocity
   */
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @class EdgeJerkHolonomic4Goal
 * @brief Edge defining the cost function for limiting the translational and rotational jerk at the end of the trajectory.
 *
 *       x              x
 *    /      \          --
 * y   |----|     y   |----|
 *     |    |       | |    | |
 *     |----|         |----|
 *    \      /          --
 * This is for holonomic type 1: 4 wheels, 45 degrees
 *                       type 2: 4 wheels, 0 degrees
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$, a goal velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [j_{w1}, j_{w2}, j_{w3}, j_{w4}]^T ) \cdot weight \f$. \n
 * \e j_{w1} \dots j_{w4} denote the normalized jerk of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 4: they represent the normalized jerk of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesJerk
 * @see EdgeJerkHolonomic4
 * @see EdgeJerkHolonomic4Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeJerkHolonomic4Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeJerkHolonomic4Goal : public BaseTebMultiEdge<4, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeJerkHolonomic4Goal()
  {
    _measurement = NULL;
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the jerk
   * @param vel_goal twist message containing the translational and rotational velocity
   */
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}; // end namespace

#endif /* EDGE_JERK_H_ */

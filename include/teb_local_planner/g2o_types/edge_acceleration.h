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

#ifndef EDGE_ACCELERATION_H_
#define EDGE_ACCELERATION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <geometry_msgs/Twist.h>

namespace teb_local_planner
{

/**
 * @class EdgeAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, \dot{omega}} ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e \dot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationStart
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() and EdgeAccelerationGoal() for defining boundary values!
 */
class EdgeAcceleration : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAcceleration()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

#ifdef USE_ANALYTIC_JACOBI
#if 0
  /*
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPointXY* conf1 = static_cast<const VertexPointXY*>(_vertices[0]);
    const VertexPointXY* conf2 = static_cast<const VertexPointXY*>(_vertices[1]);
    const VertexPointXY* conf3 = static_cast<const VertexPointXY*>(_vertices[2]);
    const VertexTimeDiff* deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexOrientation* angle1 = static_cast<const VertexOrientation*>(_vertices[5]);
    const VertexOrientation* angle2 = static_cast<const VertexOrientation*>(_vertices[6]);
    const VertexOrientation* angle3 = static_cast<const VertexOrientation*>(_vertices[7]);

    Eigen::Vector2d deltaS1 = conf2->estimate() - conf1->estimate();
    Eigen::Vector2d deltaS2 = conf3->estimate() - conf2->estimate();
    double dist1 = deltaS1.norm();
    double dist2 = deltaS2.norm();

    double sum_time = deltaT1->estimate() + deltaT2->estimate();
    double sum_time_inv = 1 / sum_time;
    double dt1_inv = 1/deltaT1->estimate();
    double dt2_inv = 1/deltaT2->estimate();
    double aux0 = 2/sum_time_inv;
    double aux1 = dist1 * deltaT1->estimate();
    double aux2 = dist2 * deltaT2->estimate();

    double vel1 = dist1 * dt1_inv;
    double vel2 = dist2 * dt2_inv;
    double omega1 = g2o::normalize_theta( angle2->estimate() - angle1->estimate() ) * dt1_inv;
    double omega2 = g2o::normalize_theta( angle3->estimate() - angle2->estimate() ) * dt2_inv;
    double acc = (vel2 - vel1) * aux0;
    double omegadot = (omega2 - omega1) * aux0;
    double aux3 = -acc/2;
    double aux4 = -omegadot/2;

    double dev_border_acc = penaltyBoundToIntervalDerivative(acc, tebConfig.robot_acceleration_max_trans,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    double dev_border_omegadot = penaltyBoundToIntervalDerivative(omegadot, tebConfig.robot_acceleration_max_rot,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);

    _jacobianOplus[0].resize(2,2); // conf1
    _jacobianOplus[1].resize(2,2); // conf2
    _jacobianOplus[2].resize(2,2); // conf3
    _jacobianOplus[3].resize(2,1); // deltaT1
    _jacobianOplus[4].resize(2,1); // deltaT2
    _jacobianOplus[5].resize(2,1); // angle1
    _jacobianOplus[6].resize(2,1); // angle2
    _jacobianOplus[7].resize(2,1); // angle3

    if (aux1==0) aux1=1e-20;
    if (aux2==0) aux2=1e-20;

    if (dev_border_acc!=0)
    {
      // TODO: double aux = aux0 * dev_border_acc;
      // double aux123 = aux / aux1;
      _jacobianOplus[0](0,0) = aux0 * deltaS1[0] / aux1 * dev_border_acc; // acc x1
      _jacobianOplus[0](0,1) = aux0 * deltaS1[1] / aux1 * dev_border_acc; // acc y1
      _jacobianOplus[1](0,0) = -aux0 * ( deltaS1[0] / aux1 + deltaS2[0] / aux2 ) * dev_border_acc; // acc x2
      _jacobianOplus[1](0,1) = -aux0 * ( deltaS1[1] / aux1 + deltaS2[1] / aux2 ) * dev_border_acc; // acc y2
      _jacobianOplus[2](0,0) = aux0 * deltaS2[0] / aux2 * dev_border_acc; // acc x3
      _jacobianOplus[2](0,1) = aux0 * deltaS2[1] / aux2 * dev_border_acc; // acc y3
      _jacobianOplus[2](0,0) = 0;
      _jacobianOplus[2](0,1) = 0;
      _jacobianOplus[3](0,0) = aux0 * (aux3 + vel1 * dt1_inv) * dev_border_acc; // acc deltaT1
      _jacobianOplus[4](0,0) = aux0 * (aux3 - vel2 * dt2_inv) * dev_border_acc; // acc deltaT2
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // acc x1
      _jacobianOplus[0](0,1) = 0; // acc y1
      _jacobianOplus[1](0,0) = 0; // acc x2
      _jacobianOplus[1](0,1) = 0; // acc y2
      _jacobianOplus[2](0,0) = 0; // acc x3
      _jacobianOplus[2](0,1) = 0; // acc y3
      _jacobianOplus[3](0,0) = 0; // acc deltaT1
      _jacobianOplus[4](0,0) = 0; // acc deltaT2
    }

    if (dev_border_omegadot!=0)
    {
      _jacobianOplus[3](1,0) = aux0 * ( aux4 + omega1 * dt1_inv ) * dev_border_omegadot; // omegadot deltaT1
      _jacobianOplus[4](1,0) = aux0 * ( aux4 - omega2 * dt2_inv ) * dev_border_omegadot; // omegadot deltaT2
      _jacobianOplus[5](1,0) = aux0 * dt1_inv * dev_border_omegadot; // omegadot angle1
      _jacobianOplus[6](1,0) = -aux0 * ( dt1_inv + dt2_inv ) * dev_border_omegadot; // omegadot angle2
      _jacobianOplus[7](1,0) = aux0 * dt2_inv * dev_border_omegadot; // omegadot angle3
    }
    else
    {
      _jacobianOplus[3](1,0) = 0; // omegadot deltaT1
      _jacobianOplus[4](1,0) = 0; // omegadot deltaT2
      _jacobianOplus[5](1,0) = 0; // omegadot angle1
      _jacobianOplus[6](1,0) = 0; // omegadot angle2
      _jacobianOplus[7](1,0) = 0; // omegadot angle3
    }

    _jacobianOplus[0](1,0) = 0; // omegadot x1
    _jacobianOplus[0](1,1) = 0; // omegadot y1
    _jacobianOplus[1](1,0) = 0; // omegadot x2
    _jacobianOplus[1](1,1) = 0; // omegadot y2
    _jacobianOplus[2](1,0) = 0; // omegadot x3
    _jacobianOplus[2](1,1) = 0; // omegadot y3
    _jacobianOplus[5](0,0) = 0; // acc angle1
    _jacobianOplus[6](0,0) = 0; // acc angle2
    _jacobianOplus[7](0,0) = 0; // acc angle3
    }
#endif
#endif

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, \dot{omega}]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
 * \e \dot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationStart()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, \dot{omega}]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
 * \e \dot{omega} is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationGoal()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic1
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a]^T ) \cdot weight \f$. \n
 * \e a denotes the normalized acceleration (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 1: a represents the normalized acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic1Start
 * @see EdgeAccelerationHolonomic1Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic1Start() and EdgeAccelerationHolonomic1Goal() for defining boundary values!
 */
class EdgeAccelerationHolonomic1 : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic1()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationHolonomic1Start
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a]^T ) \cdot weight \f$. \n
 * \e a denotes the normalized acceleration (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 1: a represents the normalized acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic1
 * @see EdgeAccelerationHolonomic1Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic1Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationHolonomic1Start : public BaseTebMultiEdge<1, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic1Start()
  {
    this->resize(3);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic1Goal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a]^T ) \cdot weight \f$. \n
 * \e a denotes the normalized acceleration (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 1: a represents the normalized acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic1
 * @see EdgeAccelerationHolonomic1Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic1Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationHolonomic1Goal : public BaseTebMultiEdge<1, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic1Goal()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic3
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
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
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w3} denote the normalized acceleration of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: they represent the normalized acceleration of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic3Start
 * @see EdgeAccelerationHolonomic3Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic3Start() and EdgeAccelerationHolonomic3Goal() for defining boundary values!
 */
class EdgeAccelerationHolonomic3 : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic3()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationHolonomic3Start
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w3} denote the normalized acceleration of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: they represent the normalized acceleration of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic3
 * @see EdgeAccelerationHolonomic3Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic3Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationHolonomic3Start : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic3Start()
  {
    this->resize(3);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic3Goal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w3} denote the normalized acceleration of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: they represent the normalized acceleration of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic3
 * @see EdgeAccelerationHolonomic3Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic3Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationHolonomic3Goal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic3Goal()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic4
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
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
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}, a_{w4}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w4} denote the normalized acceleration of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 4: they represent the normalized acceleration of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic4Start
 * @see EdgeAccelerationHolonomic4Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic4Start() and EdgeAccelerationHolonomic4Goal() for defining boundary values!
 */
class EdgeAccelerationHolonomic4 : public BaseTebMultiEdge<4, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic4()
  {
    this->resize(5);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeAccelerationHolonomic4Start
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}, a_{w4}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w4} denote the normalized acceleration of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 4: they represent the normalized acceleration of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic4
 * @see EdgeAccelerationHolonomic4Goal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic4Goal() for defining boundary values at the end of the trajectory!
 */
class EdgeAccelerationHolonomic4Start : public BaseTebMultiEdge<4, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic4Start()
  {
    this->resize(3);
    _measurement = NULL;
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
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
 * @class EdgeAccelerationHolonomic4Goal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a_{w1}, a_{w2}, a_{w3}, a_{w4}]^T ) \cdot weight \f$. \n
 * \e a_{w1} \dots a_{w4} denote the normalized acceleration of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 4: they represent the normalized acceleration of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic4
 * @see EdgeAccelerationHolonomic4Start
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomic4Start() for defining boundary (initial) values at the end of the trajectory
 */
class EdgeAccelerationHolonomic4Goal : public BaseTebMultiEdge<4, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeAccelerationHolonomic4Goal()
  {
    _measurement = NULL;
    this->resize(3);
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
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

#endif /* EDGE_ACCELERATION_H_ */

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

#ifndef EDGE_VELOCITY_H
#define EDGE_VELOCITY_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

namespace teb_local_planner
{

/**
 * @class EdgeVelocity
 * @brief Edge defining the cost function for limiting the translational and rotational velocity.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v, omega]^T ) \cdot weight \f$. \n
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocity : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

#ifdef USE_ANALYTIC_JACOBI
#if 0 //TODO the hardcoded jacobian does not include the changing direction (just the absolute value)
      // Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist*deltaT->estimate();
    double aux2 = 1/deltaT->estimate();

    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;

    double dev_border_vel = penaltyBoundToIntervalDerivative(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    double dev_border_omega = penaltyBoundToIntervalDerivative(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    _jacobianOplus[0].resize(2,3); // conf1
    _jacobianOplus[1].resize(2,3); // conf2
    _jacobianOplus[2].resize(2,1); // deltaT

//  if (aux1==0) aux1=1e-6;
//  if (aux2==0) aux2=1e-6;

    if (dev_border_vel!=0)
    {
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0,0) = -deltaS[0] * aux3; // vel x1
      _jacobianOplus[0](0,1) = -deltaS[1] * aux3; // vel y1
      _jacobianOplus[1](0,0) = deltaS[0] * aux3; // vel x2
      _jacobianOplus[1](0,1) = deltaS[1] * aux3; // vel y2
      _jacobianOplus[2](0,0) = -vel * aux2 * dev_border_vel; // vel deltaT
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // vel x1
      _jacobianOplus[0](0,1) = 0; // vel y1
      _jacobianOplus[1](0,0) = 0; // vel x2
      _jacobianOplus[1](0,1) = 0; // vel y2
      _jacobianOplus[2](0,0) = 0; // vel deltaT
    }

    if (dev_border_omega!=0)
    {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1,0) = -omega * aux4; // omega deltaT
      _jacobianOplus[0](1,2) = -aux4; // omega angle1
      _jacobianOplus[1](1,2) = aux4; // omega angle2
    }
    else
    {
      _jacobianOplus[2](1,0) = 0; // omega deltaT
      _jacobianOplus[0](1,2) = 0; // omega angle1
      _jacobianOplus[1](1,2) = 0; // omega angle2
    }

    _jacobianOplus[0](1,0) = 0; // omega x1
    _jacobianOplus[0](1,1) = 0; // omega y1
    _jacobianOplus[1](1,0) = 0; // omega x2
    _jacobianOplus[1](1,1) = 0; // omega y2
    _jacobianOplus[0](0,2) = 0; // vel angle1
    _jacobianOplus[1](0,2) = 0; // vel angle2
  }
#endif
#endif

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeVelocityHolonomic1
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
 *
 *       x
 *
 * y   |----|
 *     | ?? |
 *     |----|
 *
 * This is for Holonomic type 0: ideal
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v]^T ) \cdot weight \f$. \n
 * \e v denotes the normalized velocity (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 1: a represents the normalized velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocityHolonomic1 : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityHolonomic1()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeVelocityHolonomic3
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v_{w1}, v_{w2}, v_{w3}]^T ) \cdot weight \f$. \n
 * \e v_{w1} \dots v_{w3} denote the normalized velocity of wheel 1 to wheel 3 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: they represent the normalized velocity of 3 wheels.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocityHolonomic3 : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityHolonomic3()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


/**
 * @class EdgeVelocityHolonomic4
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
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
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v_{w1}, v_{w2}, v_{w3}, v_{w4}]^T ) \cdot weight \f$. \n
 * \e v_{w1} \dots v_{w4} denote the normalized velocity of wheel 1 to wheel 4 (computed using finite differneces). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 4: they represent the normalized velocity of 4 wheels.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeVelocityHolonomic4 : public BaseTebMultiEdge<4, double>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityHolonomic4()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError();

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


} // end namespace

#endif

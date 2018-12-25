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

#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/teb_config.h>
#include <iostream>

namespace teb_local_planner
{

inline void getVelocity(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& vel, double& omega)
{
  const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
  const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();

  double dist = deltaS.norm();
  const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
  if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
  {
      double radius =  dist/(2*sin(angle_diff/2));
      dist = fabs( angle_diff * radius ); // actual arg length!
  }
  vel = dist / deltaT->estimate();

  //vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
  vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

  omega = angle_diff / deltaT->estimate();
}

void EdgeVelocity::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocity()");
  double vel, omega;
  getVelocity(cfg_, _vertices, vel, omega);

  _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
}

inline void getVelocityHolonomic(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& vx, double& vy, double& omega)
{
  const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
  Eigen::Vector2d deltaS = conf2->position() - conf1->position();

  double cos_theta1 = std::cos(conf1->theta());
  double sin_theta1 = std::sin(conf1->theta());

  // transform conf2 into current robot frame conf1 (inverse 2d rotation matrix)
  double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
  double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();

  vx = r_dx / deltaT->estimate();
  vy = r_dy / deltaT->estimate();
  omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();
}

void EdgeVelocityHolonomic::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic()");
  double vx, vy, omega;
  getVelocityHolonomic(cfg_, _vertices, vx, vy, omega);

  _error[0] = penaltyBoundToInterval(vx, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(vy, cfg_->robot.max_vel_y, 0.0); // we do not apply the penalty epsilon here, since the velocity could be close to zero
  _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]),
                 "EdgeVelocityHolonomic::computeError() _error[0]=%f _error[1]=%f _error[2]=%f\n",_error[0],_error[1],_error[2]);
}

inline void getVelocityHolonomicNormalized(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
  Eigen::Vector2d deltaS = conf2->position() - conf1->position();

  double cos_theta1 = std::cos(conf1->theta());
  double sin_theta1 = std::sin(conf1->theta());

  // transform conf2 into current robot frame conf1 (inverse 2d rotation matrix)
  double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
  double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();

  double vx = r_dx / deltaT->estimate();
  double vy = r_dy / deltaT->estimate();
  double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();

  // normalize
  a1 = vx / cfg_->robot.max_vel_x;
  a2 = vy / cfg_->robot.max_vel_y;
  a3 = omega / cfg_->robot.max_vel_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getVelocityHolonomicNormalized(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getVelocityHolonomicNormalized(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getVelocityHolonomicNormalized(): a3=%f\n",a3);
}

void EdgeVelocityHolonomic0::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic0()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(std::hypot(std::hypot(a1,a2),a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic1::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic1()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a1+a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1+a2-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a1-a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a1-a2-a3, 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic2::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic2()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a1+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a2-a3, 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic3::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic3()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a3+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic4::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic4()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a3+a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic5::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic5()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a3-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic6::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic6()");
  double a1, a2, a3;
  getVelocityHolonomicNormalized(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(a3-a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);
}

} // end namespace


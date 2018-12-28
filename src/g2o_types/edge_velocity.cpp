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
#include <teb_local_planner/omni_helper.hpp>
#include <teb_local_planner/teb_config.h>
#include <iostream>

/*
 *
 * variable name:
 *  l: linear, a: angular
 *  v: velocity, a: acceleration, j: jerk
 *  x: axis x, y: axis y, z: axis z
 *  t: time, s: position
 *  df_x_y: f(y) - f(x)
 *
 * special case:
 *  df: f(2) - f(1), when there is no f(3) in the context
 *  x in dx: translation along axis x
 *  y in dy: translation along axis y
 *  z in dz: rotation along axis z (theta)
 *
 * examples:
 *  lvx: linear velocity in axis x
 *  aaz: angular acceleration in axis z
 *  dt_1_3: time(3) - time(1)
 *  dx: translation along axis x from time 1 to time 2
 *
 */
namespace teb_local_planner
{

inline void getVelocity2(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& lvx, double& avz)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
  const Eigen::Vector2d ds = pose2->position() - pose1->position();

  double dist = ds.norm();
  double dz = g2o::normalize_theta(pose2->theta() - pose1->theta());

  double base = dt->dt();
  if (cfg_->trajectory.exact_arc_length && std::fabs(dz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * dz) / dz);
  }

  lvx = dist / base;
  avz = dz / dt->dt();
  // consider direction
  //lvx *= g2o::sign(ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta()));
  lvx *= fast_sigmoid(100.0 * (ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta())));
}

void EdgeVelocity::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocity()");
  double lvx, avz;
  getVelocity2(cfg_, _vertices, lvx, avz);

  _error[0] = penaltyBoundToInterval(lvx, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(avz, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
}

inline void getVelocity3(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
  const Eigen::Vector2d ds = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  // transform pose2 into current robot frame pose1 (inverse 2d rotation matrix)
  double dx =  cos_theta1*ds.x() + sin_theta1*ds.y();
  double dy = -sin_theta1*ds.x() + cos_theta1*ds.y();
  double dz = g2o::normalize_theta(pose2->theta() - pose1->theta());

  double base = dt->dt();
  if (cfg_->trajectory.exact_arc_length && std::fabs(dz) > 0.05) {
      base *= 2.0 * std::sin(0.5 * dz) / dz;
  }

  double lvx = dx / base;
  double lvy = dy / base;
  double avz = dz / dt->dt();
  // normalize
  a1 = lvx / cfg_->robot.max_vel_x;
  a2 = lvy / cfg_->robot.max_vel_y;
  a3 = avz / cfg_->robot.max_vel_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getVelocity3(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getVelocity3(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getVelocity3(): a3=%f\n",a3);
}

void EdgeVelocityHolonomic0::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic0()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic1::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic1()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_1_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_1_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_1_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(error_1_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic2::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic2()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_2_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_2_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_2_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(error_2_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic3::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic3()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_3_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_3_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_3_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic4::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic4()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_4_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_4_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_4_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic5::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic5()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_5_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_5_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_5_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeVelocityHolonomic6::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeVelocityHolonomic6()");
  double a1, a2, a3;
  getVelocity3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_6_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(error_6_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(error_6_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

} // end namespace


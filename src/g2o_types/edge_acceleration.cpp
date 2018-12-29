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

#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/omni_helper.hpp>
#include <teb_local_planner/teb_config.h>

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

inline void getAcceleration2(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& lax, double& aaz)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt_1_2 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt_2_3 = static_cast<const VertexTimeDiff*>(_vertices[4]);
  const Eigen::Vector2d ds_1_2 = pose2->position() - pose1->position();
  const Eigen::Vector2d ds_2_3 = pose3->position() - pose2->position();

  double dist1 = ds_1_2.norm();
  double dist2 = ds_2_3.norm();
  double dz_1_2 = g2o::normalize_theta(pose2->theta() - pose1->theta());
  double dz_2_3 = g2o::normalize_theta(pose3->theta() - pose2->theta());

  double base1 = dt_1_2->dt();
  double base2 = dt_2_3->dt();
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(dz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * dz_1_2) / dz_1_2);
      }
      if (std::fabs(dz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * dz_2_3) / dz_2_3);
      }
  }

  double lvx1 = dist1 / base1;
  double avz1 = dz_1_2 / dt_1_2->dt();
  double lvx2 = dist2 / base2;
  double avz2 = dz_2_3 / dt_2_3->dt();
  // consider directions
  //lvx1 *= g2o::sign(ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta()));
  //lvx2 *= g2o::sign(ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta()));
  lvx1 *= fast_sigmoid(100.0 * (ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta())));
  lvx2 *= fast_sigmoid(100.0 * (ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta())));

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  lax  = (lvx2 - lvx1) / dt2_1_3;
  aaz  = (avz2 - avz1) / dt2_1_3;
}

inline void getAcceleration2Start(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& lax, double& aaz)
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

  double lvx1 = _measurement->linear.x;
  double avz1 = _measurement->angular.z;
  double lvx2 = dist / base;
  double avz2 = dz / dt->dt();
  // consider directions
  //lvx *= g2o::sign(ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta()));
  lvx2 *= fast_sigmoid(100.0 * (ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta())));

  double dt2 = 0.5 * dt->dt();
  lax  = (lvx2 - lvx1) / dt2;
  aaz  = (avz2 - avz1) / dt2;
}

inline void getAcceleration2Goal(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& lax, double& aaz)
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

  double lvx1 = dist / base;
  double avz1 = dz / dt->dt();
  double lvx2 = _measurement->linear.x;
  double avz2 = _measurement->angular.z;
  // consider directions
  //lvx *= g2o::sign(ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta()));
  lvx2 *= fast_sigmoid(100.0 * (ds.x() * cos(pose1->theta()) + ds.y() * sin(pose1->theta())));

  double dt2 = 0.5 * dt->dt();
  lax  = (lvx2 - lvx1) / dt2;
  aaz  = (avz2 - avz1) / dt2;
}

void EdgeAcceleration::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAcceleration()");
  double lax, aaz;
  getAcceleration2(cfg_, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeAccelerationStart::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
  double lax, aaz;
  getAcceleration2Start(cfg_, _measurement, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeAccelerationGoal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
  double lax, aaz;
  getAcceleration2Goal(cfg_, _measurement, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
}

inline void getAcceleration3(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt_1_2 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt_2_3 = static_cast<const VertexTimeDiff*>(_vertices[4]);
  const Eigen::Vector2d ds_1_2 = pose2->position() - pose1->position();
  const Eigen::Vector2d ds_2_3 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());
  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double dx_1_2 =  cos_theta1*ds_1_2.x() + sin_theta1*ds_1_2.y();
  double dy_1_2 = -sin_theta1*ds_1_2.x() + cos_theta1*ds_1_2.y();
  double dz_1_2 = g2o::normalize_theta(pose2->theta() - pose1->theta());
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double dx_2_3 =  cos_theta2*ds_2_3.x() + sin_theta2*ds_2_3.y();
  double dy_2_3 = -sin_theta2*ds_2_3.x() + cos_theta2*ds_2_3.y();
  double dz_2_3 = g2o::normalize_theta(pose3->theta() - pose2->theta());

  double base1 = dt_1_2->dt();
  double base2 = dt_2_3->dt();
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(dz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * dz_1_2) / dz_1_2);
      }
      if (std::fabs(dz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * dz_2_3) / dz_2_3);
      }
  }

  double lvx1 = dx_1_2 / base1;
  double lvy1 = dy_1_2 / base1;
  double avz1 = dz_1_2 / dt_1_2->dt();
  double lvx2 = dx_2_3 / base2;
  double lvy2 = dy_2_3 / base2;
  double avz2 = dz_2_3 / dt_2_3->dt();

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dlvx  = lvx2 - lvx1;
  double dlvy  = lvy2 - lvy1;
  double davz  = avz2 - avz1;

  double base = dt2_1_3;
  if (cfg_->trajectory.exact_arc_length && std::fabs(davz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * davz) / davz);
  }

  double lax = dlvx / base;
  double lay = dlvy / base;
  double aaz = davz / dt2_1_3;
  // normalize
  a1 = lax / cfg_->robot.acc_lim_x;
  a2 = lay / cfg_->robot.acc_lim_y;
  a3 = aaz / cfg_->robot.acc_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getAcceleration3(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getAcceleration3(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getAcceleration3(): a3=%f\n",a3);
}

inline void getAcceleration3Start(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
  const Eigen::Vector2d ds = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double dx =  cos_theta1*ds.x() + sin_theta1*ds.y();
  double dy = -sin_theta1*ds.x() + cos_theta1*ds.y();
  double dz = g2o::normalize_theta(pose2->theta() - pose1->theta());

  double base = dt->dt();
  if (cfg_->trajectory.exact_arc_length && std::fabs(dz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * dz) / dz);
  }

  double lvx1 = _measurement->linear.x;
  double lvy1 = _measurement->linear.y;
  double avz1 = _measurement->angular.z;
  double lvx2 = dx / base;
  double lvy2 = dy / base;
  double avz2 = dz / dt->dt();

  double dt2 = 0.5 * dt->dt();
  double dlvx  = lvx2 - lvx1;
  double dlvy  = lvy2 - lvy1;
  double davz  = avz2 - avz1;

  base = dt2;
  if (cfg_->trajectory.exact_arc_length && std::fabs(davz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * davz) / davz);
  }

  double lax = dlvx / base;
  double lay = dlvy / base;
  double aaz = davz / dt2;
  // normalize
  a1 = lax / cfg_->robot.acc_lim_x;
  a2 = lay / cfg_->robot.acc_lim_y;
  a3 = aaz / cfg_->robot.acc_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getAcceleration3Start(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getAcceleration3Start(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getAcceleration3Start(): a3=%f\n",a3);
}

inline void getAcceleration3Goal(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);
  const Eigen::Vector2d ds = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double dx =  cos_theta1*ds.x() + sin_theta1*ds.y();
  double dy = -sin_theta1*ds.x() + cos_theta1*ds.y();
  double dz = g2o::normalize_theta(pose2->theta() - pose1->theta());

  double base = dt->dt();
  if (cfg_->trajectory.exact_arc_length && std::fabs(dz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * dz) / dz);
  }

  double lvx1 = dx / base;
  double lvy1 = dy / base;
  double avz1 = dz / dt->dt();
  double lvx2 = _measurement->linear.x;
  double lvy2 = _measurement->linear.y;
  double avz2 = _measurement->angular.z;

  double dt2 = 0.5 * dt->dt();
  double dlvx  = lvx2 - lvx1;
  double dlvy  = lvy2 - lvy1;
  double davz  = avz2 - avz1;

  base = dt2;
  if (cfg_->trajectory.exact_arc_length && std::fabs(davz) > 0.05) {
      base *= std::fabs(2.0 * std::sin(0.5 * davz) / davz);
  }

  double lax = dlvx / base;
  double lay = dlvy / base;
  double aaz = davz / dt2;
  // normalize
  a1 = lax / cfg_->robot.acc_lim_x;
  a2 = lay / cfg_->robot.acc_lim_y;
  a3 = aaz / cfg_->robot.acc_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getAcceleration3Goal(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getAcceleration3Goal(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getAcceleration3Goal(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic1::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic1()");
  double a1, a2, a3;
  getAcceleration3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeAccelerationHolonomic1Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic1Start()");
  double a1, a2, a3;
  getAcceleration3Start(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeAccelerationHolonomic1Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic1Goal()");
  double a1, a2, a3;
  getAcceleration3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeAccelerationHolonomic3::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic3()");
  double a1, a2, a3;
  getAcceleration3(cfg_, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 3:
      _error[0] = penaltyBoundToInterval(error_3_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_3_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_3_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 4:
      _error[0] = penaltyBoundToInterval(error_4_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_4_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_4_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 5:
      _error[0] = penaltyBoundToInterval(error_5_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_5_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_5_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 6:
      _error[0] = penaltyBoundToInterval(error_6_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_6_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_6_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

void EdgeAccelerationHolonomic3Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic3Start()");
  double a1, a2, a3;
  getAcceleration3Start(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 3:
      _error[0] = penaltyBoundToInterval(error_3_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_3_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_3_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 4:
      _error[0] = penaltyBoundToInterval(error_4_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_4_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_4_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 5:
      _error[0] = penaltyBoundToInterval(error_5_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_5_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_5_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 6:
      _error[0] = penaltyBoundToInterval(error_6_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_6_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_6_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

void EdgeAccelerationHolonomic3Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic3Goal()");
  double a1, a2, a3;
  getAcceleration3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 3:
      _error[0] = penaltyBoundToInterval(error_3_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_3_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_3_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 4:
      _error[0] = penaltyBoundToInterval(error_4_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_4_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_4_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 5:
      _error[0] = penaltyBoundToInterval(error_5_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_5_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_5_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 6:
      _error[0] = penaltyBoundToInterval(error_6_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_6_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_6_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

void EdgeAccelerationHolonomic4::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic4()");
  double a1, a2, a3;
  getAcceleration3(cfg_, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 1:
      _error[0] = penaltyBoundToInterval(error_1_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_1_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_1_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_1_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 2:
      _error[0] = penaltyBoundToInterval(error_2_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_2_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_2_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_2_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

void EdgeAccelerationHolonomic4Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic4Start()");
  double a1, a2, a3;
  getAcceleration3Start(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 1:
      _error[0] = penaltyBoundToInterval(error_1_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_1_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_1_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_1_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 2:
      _error[0] = penaltyBoundToInterval(error_2_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_2_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_2_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_2_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

void EdgeAccelerationHolonomic4Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic4Goal()");
  double a1, a2, a3;
  getAcceleration3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  switch (cfg_->robot.omni_type) {
  case 1:
      _error[0] = penaltyBoundToInterval(error_1_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_1_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_1_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_1_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  case 2:
      _error[0] = penaltyBoundToInterval(error_2_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[1] = penaltyBoundToInterval(error_2_2(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[2] = penaltyBoundToInterval(error_2_3(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      _error[3] = penaltyBoundToInterval(error_2_4(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
      break;
  }
}

}; // end namespace


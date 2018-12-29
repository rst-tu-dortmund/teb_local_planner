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

#include <teb_local_planner/g2o_types/edge_jerk.h>
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

inline void getJerk2(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& ljx, double& ajz)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
  const VertexTimeDiff* dt_1_2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
  const VertexTimeDiff* dt_2_3 = static_cast<const VertexTimeDiff*>(_vertices[5]);
  const VertexTimeDiff* dt_3_4 = static_cast<const VertexTimeDiff*>(_vertices[6]);
  const Eigen::Vector2d ds_1_2 = pose2->position() - pose1->position();
  const Eigen::Vector2d ds_2_3 = pose3->position() - pose2->position();
  const Eigen::Vector2d ds_3_4 = pose4->position() - pose3->position();

  double dist1 = ds_1_2.norm();
  double dist2 = ds_2_3.norm();
  double dist3 = ds_3_4.norm();
  double dz_1_2 = g2o::normalize_theta(pose2->theta() - pose1->theta());
  double dz_2_3 = g2o::normalize_theta(pose3->theta() - pose2->theta());
  double dz_3_4 = g2o::normalize_theta(pose4->theta() - pose3->theta());

  double base1 = dt_1_2->dt();
  double base2 = dt_2_3->dt();
  double base3 = dt_3_4->dt();
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(dz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * dz_1_2) / dz_1_2);
      }
      if (std::fabs(dz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * dz_2_3) / dz_2_3);
      }
      if (std::fabs(dz_3_4) > 0.05) {
          base3 *= std::fabs(2.0 * std::sin(0.5 * dz_3_4) / dz_3_4);
      }
  }

  double lvx1 = dist1 / base1;
  double avz1 = dz_1_2 / dt_1_2->dt();
  double lvx2 = dist2 / base2;
  double avz2 = dz_2_3 / dt_2_3->dt();
  double lvx3 = dist3 / base3;
  double avz3 = dz_3_4 / dt_3_4->dt();
  // consider directions
  //lvx1 *= g2o::sign(ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta()));
  //lvx2 *= g2o::sign(ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta()));
  //lvx3 *= g2o::sign(ds_3_4.x() * cos(pose3->theta()) + ds_3_4.y() * sin(pose3->theta()));
  lvx1 *= fast_sigmoid(100.0 * (ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta())));
  lvx2 *= fast_sigmoid(100.0 * (ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta())));
  lvx3 *= fast_sigmoid(100.0 * (ds_3_4.x() * cos(pose3->theta()) + ds_3_4.y() * sin(pose3->theta())));

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dt2_2_4 = 0.5 * (dt_2_3->dt() + dt_3_4->dt());
  double lax1  = (lvx2 - lvx1) / dt2_1_3;
  double aaz1  = (avz2 - avz1) / dt2_1_3;
  double lax2  = (lvx3 - lvx2) / dt2_2_4;
  double aaz2  = (avz3 - avz2) / dt2_2_4;

  double dt4_1_4 = 0.5 * (dt2_1_3 + dt2_2_4);
  ljx = (lax2 - lax1) / dt4_1_4;
  ajz = (aaz2 - aaz1) / dt4_1_4;
}

inline void getJerk2Start(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& ljx, double& ajz)
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

  double lvx1 = _measurement->linear.x;
  double avz1 = _measurement->angular.z;
  double lvx2 = dist1 / base1;
  double avz2 = dz_1_2 / dt_1_2->dt();
  double lvx3 = dist2 / base2;
  double avz3 = dz_2_3 / dt_2_3->dt();
  // consider directions
  //lvx2 *= g2o::sign(ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta()));
  //lvx3 *= g2o::sign(ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta()));
  lvx2 *= fast_sigmoid(100.0 * (ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta())));
  lvx3 *= fast_sigmoid(100.0 * (ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta())));

  double dt2_1_2 = 0.5 * dt_1_2->dt();
  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double lax1  = (lvx2 - lvx1) / dt2_1_2;
  double aaz1  = (avz2 - avz1) / dt2_1_2;
  double lax2  = (lvx3 - lvx2) / dt2_1_3;
  double aaz2  = (avz3 - avz2) / dt2_1_3;

  // assume lax0, aaz0 = 0
  double dt4_1_2 = 0.5 * dt2_1_2;
  double dt4_1_3 = 0.5 * (dt2_1_2 + dt2_1_3);
  double ljx1 = lax1 / dt4_1_2;
  double ajz1 = aaz1 / dt4_1_2;
  double ljx2 = (lax2 - lax1) / dt4_1_3;
  double ajz2 = (aaz2 - aaz1) / dt4_1_3;

  ljx = (std::fabs(ljx1) > std::fabs(ljx2)) ? ljx1 : ljx2;
  ajz = (std::fabs(ajz1) > std::fabs(ajz2)) ? ajz1 : ajz2;
}

inline void getJerk2Goal(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& ljx, double& ajz)
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
  double lvx3 = _measurement->linear.x;
  double avz3 = _measurement->angular.z;
  // consider directions
  //lvx1 *= g2o::sign(ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta()));
  //lvx2 *= g2o::sign(ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta()));
  lvx1 *= fast_sigmoid(100.0 * (ds_1_2.x() * cos(pose1->theta()) + ds_1_2.y() * sin(pose1->theta())));
  lvx2 *= fast_sigmoid(100.0 * (ds_2_3.x() * cos(pose2->theta()) + ds_2_3.y() * sin(pose2->theta())));

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dt2_2_3 = 0.5 * dt_2_3->dt();
  double lax1  = (lvx2 - lvx1) / dt2_1_3;
  double aaz1  = (avz2 - avz1) / dt2_1_3;
  double lax2  = (lvx3 - lvx2) / dt2_2_3;
  double aaz2  = (avz3 - avz2) / dt2_2_3;

  // assume lax3, aaz3 = 0
  double dt4_1_3 = 0.5 * (dt2_1_3 + dt2_2_3);
  double dt4_2_3 = 0.5 * dt2_2_3;
  double ljx1 = (lax2 - lax1) / dt4_1_3;
  double ajz1 = (aaz2 - aaz1) / dt4_1_3;
  double ljx2 = -lax2 / dt4_2_3;
  double ajz2 = -aaz2 / dt4_2_3;

  ljx = (std::fabs(ljx1) > std::fabs(ljx2)) ? ljx1 : ljx2;
  ajz = (std::fabs(ajz1) > std::fabs(ajz2)) ? ajz1 : ajz2;
}

void EdgeJerk::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeJerk()");
  double lax, aaz;
  getJerk2(cfg_, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerk::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerk::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeJerkStart::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkStart()");
  double lax, aaz;
  getJerk2Start(cfg_, _measurement, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerkStart::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerkStart::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeJerkGoal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkGoal()");
  double lax, aaz;
  getJerk2Goal(cfg_, _measurement, _vertices, lax, aaz);

  _error[0] = penaltyBoundToInterval(lax,cfg_->robot.jerk_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(aaz,cfg_->robot.jerk_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeJerkGoal::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeJerkGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
}

inline void getJerk3(const TebConfig* cfg_, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
{
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexPose* pose4 = static_cast<const VertexPose*>(_vertices[3]);
  const VertexTimeDiff* dt_1_2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
  const VertexTimeDiff* dt_2_3 = static_cast<const VertexTimeDiff*>(_vertices[5]);
  const VertexTimeDiff* dt_3_4 = static_cast<const VertexTimeDiff*>(_vertices[6]);
  const Eigen::Vector2d ds_1_2 = pose2->position() - pose1->position();
  const Eigen::Vector2d ds_2_3 = pose3->position() - pose2->position();
  const Eigen::Vector2d ds_3_4 = pose4->position() - pose3->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());
  double cos_theta3 = std::cos(pose3->theta());
  double sin_theta3 = std::sin(pose3->theta());
  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double dx_1_2 =  cos_theta1*ds_1_2.x() + sin_theta1*ds_1_2.y();
  double dy_1_2 = -sin_theta1*ds_1_2.x() + cos_theta1*ds_1_2.y();
  double dz_1_2 = g2o::normalize_theta(pose2->theta() - pose1->theta());
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double dx_2_3 =  cos_theta2*ds_2_3.x() + sin_theta2*ds_2_3.y();
  double dy_2_3 = -sin_theta2*ds_2_3.x() + cos_theta2*ds_2_3.y();
  double dz_2_3 = g2o::normalize_theta(pose3->theta() - pose2->theta());
  // transform pose4 into robot frame pose3 (inverse 2d rotation matrix)
  double dx_3_4 =  cos_theta3*ds_3_4.x() + sin_theta3*ds_3_4.y();
  double dy_3_4 = -sin_theta3*ds_3_4.x() + cos_theta3*ds_3_4.y();
  double dz_3_4 = g2o::normalize_theta(pose4->theta() - pose3->theta());

  double base1 = dt_1_2->dt();
  double base2 = dt_2_3->dt();
  double base3 = dt_3_4->dt();
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(dz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * dz_1_2) / dz_1_2);
      }
      if (std::fabs(dz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * dz_2_3) / dz_2_3);
      }
      if (std::fabs(dz_3_4) > 0.05) {
          base3 *= std::fabs(2.0 * std::sin(0.5 * dz_3_4) / dz_3_4);
      }
  }

  double lvx1 = dx_1_2 / base1;
  double lvy1 = dy_1_2 / base1;
  double avz1 = dz_1_2 / dt_1_2->dt();
  double lvx2 = dx_2_3 / base2;
  double lvy2 = dy_2_3 / base2;
  double avz2 = dz_2_3 / dt_2_3->dt();
  double lvx3 = dx_3_4 / base3;
  double lvy3 = dy_3_4 / base3;
  double avz3 = dz_3_4 / dt_3_4->dt();

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dt2_2_4 = 0.5 * (dt_2_3->dt() + dt_3_4->dt());
  double dlvx_1_2  = lvx2 - lvx1;
  double dlvy_1_2  = lvy2 - lvy1;
  double davz_1_2  = avz2 - avz1;
  double dlvx_2_3  = lvx3 - lvx2;
  double dlvy_2_3  = lvy3 - lvy2;
  double davz_2_3  = avz3 - avz2;

  base1 = dt2_1_3;
  base2 = dt2_2_4;
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(davz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * davz_1_2) / davz_1_2);
      }
      if (std::fabs(davz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * davz_2_3) / davz_2_3);
      }
  }

  double lax1 = dlvx_1_2 / base1;
  double lay1 = dlvy_1_2 / base1;
  double aaz1 = davz_1_2 / dt2_1_3;
  double lax2 = dlvx_2_3 / base2;
  double lay2 = dlvy_2_3 / base2;
  double aaz2 = davz_2_3 / dt2_2_4;

  double dt4_1_4 = 0.5 * (dt2_1_3 + dt2_2_4);
  double dlax  = lax2 - lax1;
  double dlay  = lay2 - lay1;
  double daaz  = aaz2 - aaz1;

  base1 = dt4_1_4;
  if (cfg_->trajectory.exact_arc_length && std::fabs(daaz) > 0.05) {
      base1 *= std::fabs(2.0 * std::sin(0.5 * daaz) / daaz);
  }

  double ljx = dlax / base1;
  double ljy = dlay / base1;
  double ajz = daaz / dt4_1_4;
  // normalize
  a1 = ljx / cfg_->robot.jerk_lim_x;
  a2 = ljy / cfg_->robot.jerk_lim_y;
  a3 = ajz / cfg_->robot.jerk_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getJerk3(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getJerk3(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getJerk3(): a3=%f\n",a3);
}

inline void getJerk3Start(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
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
  double dz_2_3 = g2o::normalize_theta(pose2->theta() - pose1->theta());

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

  double lvx1 = _measurement->linear.x;
  double lvy1 = _measurement->linear.y;
  double avz1 = _measurement->angular.z;
  double lvx2 = dx_1_2 / base1;
  double lvy2 = dy_1_2 / base1;
  double avz2 = dz_1_2 / dt_1_2->dt();
  double lvx3 = dx_2_3 / base2;
  double lvy3 = dy_2_3 / base2;
  double avz3 = dz_2_3 / dt_2_3->dt();

  double dt2_1_2 = 0.5 * dt_1_2->dt();
  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dlvx_1_2  = lvx2 - lvx1;
  double dlvy_1_2  = lvy2 - lvy1;
  double davz_1_2  = avz2 - avz1;
  double dlvx_2_3  = lvx3 - lvx2;
  double dlvy_2_3  = lvy3 - lvy2;
  double davz_2_3  = avz3 - avz2;

  base1 = dt2_1_2;
  base2 = dt2_1_3;
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(davz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * davz_1_2) / davz_1_2);
      }
      if (std::fabs(davz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * davz_2_3) / davz_2_3);
      }
  }

  double lax1 = dlvx_1_2 / base1;
  double lay1 = dlvy_1_2 / base1;
  double aaz1 = davz_1_2 / dt2_1_2;
  double lax2 = dlvx_2_3 / base2;
  double lay2 = dlvy_2_3 / base2;
  double aaz2 = davz_2_3 / dt2_1_3;

  // assume lax0, aaz0 = 0
  double dt4_1_2 = 0.5 * dt2_1_2;
  double dt4_1_3 = 0.5 * (dt2_1_2 + dt2_1_3);
  double dlax_0_1  = lax1;
  double dlay_0_1  = lay1;
  double daaz_0_1  = aaz1;
  double dlax_1_2  = lax2 - lax1;
  double dlay_1_2  = lay2 - lay1;
  double daaz_1_2  = aaz2 - aaz1;

  base1 = dt4_1_2;
  base2 = dt4_1_3;
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(daaz_0_1) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * daaz_0_1) / daaz_0_1);
      }
      if (std::fabs(daaz_1_2) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * daaz_1_2) / daaz_1_2);
      }
  }

  double ljx1 = dlax_0_1 / base1;
  double ljy1 = dlay_0_1 / base1;
  double ajz1 = daaz_0_1 / dt4_1_2;
  double ljx2 = dlax_1_2 / base2;
  double ljy2 = dlay_1_2 / base2;
  double ajz2 = daaz_1_2 / dt4_1_3;
  double ljx = (std::fabs(ljx1) > std::fabs(ljx2)) ? ljx1 : ljx2;
  double ljy = (std::fabs(ljy1) > std::fabs(ljy2)) ? ljy1 : ljy2;
  double ajz = (std::fabs(ajz1) > std::fabs(ajz2)) ? ajz1 : ajz2;
  // normalize
  a1 = ljx / cfg_->robot.jerk_lim_x;
  a2 = ljy / cfg_->robot.jerk_lim_y;
  a3 = ajz / cfg_->robot.jerk_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getJerk3Start(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getJerk3Start(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getJerk3Start(): a3=%f\n",a3);
}

inline void getJerk3Goal(const TebConfig* cfg_, const geometry_msgs::Twist* _measurement, const g2o::HyperGraph::VertexContainer _vertices, double& a1, double& a2, double& a3)
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
  double dz_2_3 = g2o::normalize_theta(pose2->theta() - pose1->theta());

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
  double lvx3 = _measurement->linear.x;
  double lvy3 = _measurement->linear.y;
  double avz3 = _measurement->angular.z;

  double dt2_1_3 = 0.5 * (dt_1_2->dt() + dt_2_3->dt());
  double dt2_2_3 = 0.5 * dt_2_3->dt();
  double dlvx_1_2  = lvx2 - lvx1;
  double dlvy_1_2  = lvy2 - lvy1;
  double davz_1_2  = avz2 - avz1;
  double dlvx_2_3  = lvx3 - lvx2;
  double dlvy_2_3  = lvy3 - lvy2;
  double davz_2_3  = avz3 - avz2;

  base1 = dt2_1_3;
  base2 = dt2_2_3;
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(davz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * davz_1_2) / davz_1_2);
      }
      if (std::fabs(davz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * davz_2_3) / davz_2_3);
      }
  }

  double lax1 = dlvx_1_2 / base1;
  double lay1 = dlvy_1_2 / base1;
  double aaz1 = davz_1_2 / dt2_1_3;
  double lax2 = dlvx_2_3 / base2;
  double lay2 = dlvy_2_3 / base2;
  double aaz2 = davz_2_3 / dt2_2_3;

  // assume lax3, aaz3 = 0
  double dt4_1_3 = 0.5 * (dt2_1_3 + dt2_2_3);
  double dt4_2_3 = 0.5 * dt2_2_3;
  double dlax_1_2  = lax2 - lax1;
  double dlay_1_2  = lay2 - lay1;
  double daaz_1_2  = aaz2 - aaz1;
  double dlax_2_3  = -lax2;
  double dlay_2_3  = -lay2;
  double daaz_2_3  = -aaz2;

  base1 = dt4_1_3;
  base2 = dt4_2_3;
  if (cfg_->trajectory.exact_arc_length) {
      if (std::fabs(daaz_1_2) > 0.05) {
          base1 *= std::fabs(2.0 * std::sin(0.5 * daaz_1_2) / daaz_1_2);
      }
      if (std::fabs(daaz_2_3) > 0.05) {
          base2 *= std::fabs(2.0 * std::sin(0.5 * daaz_2_3) / daaz_2_3);
      }
  }

  double ljx1 = dlax_1_2 / base1;
  double ljy1 = dlay_1_2 / base1;
  double ajz1 = daaz_1_2 / dt4_1_3;
  double ljx2 = dlax_2_3 / base2;
  double ljy2 = dlay_2_3 / base2;
  double ajz2 = daaz_2_3 / dt4_2_3;
  double ljx = (std::fabs(ljx1) > std::fabs(ljx2)) ? ljx1 : ljx2;
  double ljy = (std::fabs(ljy1) > std::fabs(ljy2)) ? ljy1 : ljy2;
  double ajz = (std::fabs(ajz1) > std::fabs(ajz2)) ? ajz1 : ajz2;
  // normalize
  a1 = ljx / cfg_->robot.jerk_lim_x;
  a2 = ljy / cfg_->robot.jerk_lim_y;
  a3 = ajz / cfg_->robot.jerk_lim_theta;

  ROS_ASSERT_MSG(std::isfinite(a1), "getJerk3Goal(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "getJerk3Goal(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "getJerk3Goal(): a3=%f\n",a3);
}

void EdgeJerkHolonomic1::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeJerkHolonomic1()");
  double a1, a2, a3;
  getJerk3(cfg_, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeJerkHolonomic1Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkHolonomic1Start()");
  double a1, a2, a3;
  getJerk3Start(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeJerkHolonomic1Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkHolonomic1Goal()");
  double a1, a2, a3;
  getJerk3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

  // error
  _error[0] = penaltyBoundToInterval(error_0_1(a1, a2, a3), 1.0, cfg_->optim.penalty_epsilon);
}

void EdgeJerkHolonomic3::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeJerkHolonomic3()");
  double a1, a2, a3;
  getJerk3(cfg_, _vertices, a1, a2, a3);

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

void EdgeJerkHolonomic3Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkHolonomic3Start()");
  double a1, a2, a3;
  getJerk3Start(cfg_, _measurement, _vertices, a1, a2, a3);

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

void EdgeJerkHolonomic3Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkHolonomic3Goal()");
  double a1, a2, a3;
  getJerk3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

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

void EdgeJerkHolonomic4::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeJerkHolonomic4()");
  double a1, a2, a3;
  getJerk3(cfg_, _vertices, a1, a2, a3);

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

void EdgeJerkHolonomic4Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeJerkHolonomic4Start()");
  double a1, a2, a3;
  getJerk3Start(cfg_, _measurement, _vertices, a1, a2, a3);

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

void EdgeJerkHolonomic4Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeJerkHolonomic4Goal()");
  double a1, a2, a3;
  getJerk3Goal(cfg_, _measurement, _vertices, a1, a2, a3);

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


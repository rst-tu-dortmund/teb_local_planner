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
#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

void EdgeAcceleration::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAcceleration()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  const Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double dist1 = diff1.norm();
  double dist2 = diff2.norm();
  const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
  const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());

  if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
  {
      if (angle_diff1 != 0)
      {
          const double radius =  dist1/(2*sin(angle_diff1/2));
          dist1 = fabs( angle_diff1 * radius ); // actual arg length!
      }
      if (angle_diff2 != 0)
      {
          const double radius =  dist2/(2*sin(angle_diff2/2));
          dist2 = fabs( angle_diff2 * radius ); // actual arg length!
      }
  }

  double vel1 = dist1 / dt1->dt();
  double vel2 = dist2 / dt2->dt();

  // consider directions
     vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta()));
     vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta()));
  vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) );
  vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) );

  const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );

  _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  const double omega1 = angle_diff1 / dt1->dt();
  const double omega2 = angle_diff2 / dt2->dt();
  const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );

  _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeAccelerationStart::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  const Eigen::Vector2d diff = pose2->position() - pose1->position();
  double dist = diff.norm();
  const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());
  if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
  {
      const double radius =  dist/(2*sin(angle_diff/2));
      dist = fabs( angle_diff * radius ); // actual arg length!
  }

  const double vel1 = _measurement->linear.x;
  double vel2 = dist / dt->dt();

  // consider directions
  //vel2 *= g2o::sign(diff[0]*cos(pose1->theta()) + diff[1]*sin(pose1->theta()));
  vel2 *= fast_sigmoid( 100*(diff.x()*cos(pose1->theta()) + diff.y()*sin(pose1->theta())) );

  const double acc_lin  = (vel2 - vel1) / dt->dt();

  _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  const double omega1 = _measurement->angular.z;
  const double omega2 = angle_diff / dt->dt();
  const double acc_rot  = (omega2 - omega1) / dt->dt();

  _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeAccelerationGoal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  const Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();
  double dist = diff.norm();
  const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
  if (cfg_->trajectory.exact_arc_length  && angle_diff != 0)
  {
      double radius =  dist/(2*sin(angle_diff/2));
      dist = fabs( angle_diff * radius ); // actual arg length!
  }

  double vel1 = dist / dt->dt();
  const double vel2 = _measurement->linear.x;

  // consider directions
  //vel1 *= g2o::sign(diff[0]*cos(pose_pre_goal->theta()) + diff[1]*sin(pose_pre_goal->theta()));
  vel1 *= fast_sigmoid( 100*(diff.x()*cos(pose_pre_goal->theta()) + diff.y()*sin(pose_pre_goal->theta())) );

  const double acc_lin  = (vel2 - vel1) / dt->dt();

  _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  const double omega1 = angle_diff / dt->dt();
  const double omega2 = _measurement->angular.z;
  const double acc_rot  = (omega2 - omega1) / dt->dt();

  _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
}

void EdgeAccelerationHolonomic::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  _error[0] = penaltyBoundToInterval(acc_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(acc_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationHolonomic::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationHolonomic::computeError() strafing: _error[1]=%f\n",_error[1]);
  ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationHolonomic::computeError() rotational: _error[2]=%f\n",_error[2]);
}

void EdgeAccelerationHolonomicStart::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomicStart()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationHolonomicStart::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationHolonomicStart::computeError() strafing: _error[1]=%f\n",_error[1]);
  ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationHolonomicStart::computeError() rotational: _error[2]=%f\n",_error[2]);
}

void EdgeAccelerationHolonomicGoal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomicGoal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationHolonomicGoal::computeError() translational: _error[0]=%f\n",_error[0]);
  ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationHolonomicGoal::computeError() strafing: _error[1]=%f\n",_error[1]);
  ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationHolonomicGoal::computeError() rotational: _error[2]=%f\n",_error[2]);
}

void EdgeAccelerationHolonomic0::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic0()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(std::hypot(std::hypot(a1,a2),a3), 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic0::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic0::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic0::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic0Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic0Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(std::hypot(std::hypot(a1,a2),a3), 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic0Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic0Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic0Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic0Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic0Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(std::hypot(std::hypot(a1,a2),a3), 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic0Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic0Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic0Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic1::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic1()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1+a2-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a1-a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a1-a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic1::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic1::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic1::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic1Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic1Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1+a2-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a1-a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a1-a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic1Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic1Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic1Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic1Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic1Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1+a2-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a1-a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a1-a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic1Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic1Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic1Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic2::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic2()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic2::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic2::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic2::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic2Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic2Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic2Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic2Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic2Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic2Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic2Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a1+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a1-a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a2+a3, 1.0, cfg_->optim.penalty_epsilon);
  _error[3] = penaltyBoundToInterval(a2-a3, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic2Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic2Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic2Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic3::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic3()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic3::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic3::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic3::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic3Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic3Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic3Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic3Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic3Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic3Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic3Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1-a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic3Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic3Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic3Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic4::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic4()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic4::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic4::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic4::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic4Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic4Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic4Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic4Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic4Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic4Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic4Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3+a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3-a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic4Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic4Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic4Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic5::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic5()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic5::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic5::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic5::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic5Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic5Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic5Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic5Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic5Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic5Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic5Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3-a1+a2*0.5, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic5Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic5Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic5Goal::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic6::computeError()
{
  ROS_ASSERT_MSG(cfg_, "You must call setTebConfig() on EdgeAccelerationHolonomic6()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
  const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
  const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff1 = pose2->position() - pose1->position();
  Eigen::Vector2d diff2 = pose3->position() - pose2->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());
  double cos_theta2 = std::cos(pose2->theta());
  double sin_theta2 = std::sin(pose2->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
  double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
  // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
  double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
  double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();

  double vel1_x = p1_dx / dt1->dt();
  double vel1_y = p1_dy / dt1->dt();
  double vel2_x = p2_dx / dt2->dt();
  double vel2_y = p2_dy / dt2->dt();

  double dt12 = dt1->dt() + dt2->dt();

  double acc_x  = (vel2_x - vel1_x)*2 / dt12;
  double acc_y  = (vel2_y - vel1_y)*2 / dt12;

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
  double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
  double acc_rot  = (omega2 - omega1)*2 / dt12;

  // normalize
  double a1 = acc_x / cfg_->robot.acc_lim_x;
  double a2 = acc_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic6::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic6::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic6::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic6Start::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationHolonomic6Start()");
  const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION
  Eigen::Vector2d diff = pose2->position() - pose1->position();

  double cos_theta1 = std::cos(pose1->theta());
  double sin_theta1 = std::sin(pose1->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = _measurement->linear.x;
  double vel1_y = _measurement->linear.y;
  double vel2_x = p1_dx / dt->dt();
  double vel2_y = p1_dy / dt->dt();

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = _measurement->angular.z;
  double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic6Start::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic6Start::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic6Start::computeError(): a3=%f\n",a3);
}

void EdgeAccelerationHolonomic6Goal::computeError()
{
  ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationHolonomic6Goal()");
  const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
  const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
  const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

  // VELOCITY & ACCELERATION

  Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();

  double cos_theta1 = std::cos(pose_pre_goal->theta());
  double sin_theta1 = std::sin(pose_pre_goal->theta());

  // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
  double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
  double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();

  double vel1_x = p1_dx / dt->dt();
  double vel1_y = p1_dy / dt->dt();
  double vel2_x = _measurement->linear.x;
  double vel2_y = _measurement->linear.y;

  double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
  double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

  // ANGULAR ACCELERATION
  double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
  double omega2 = _measurement->angular.z;
  double acc_rot  = (omega2 - omega1) / dt->dt();

  // normalize
  double a1 = acc_lin_x / cfg_->robot.acc_lim_x;
  double a2 = acc_lin_y / cfg_->robot.acc_lim_y;
  double a3 = acc_rot / cfg_->robot.acc_lim_theta;

  // error
  _error[0] = penaltyBoundToInterval(a3-a1, 1.0, cfg_->optim.penalty_epsilon);
  _error[1] = penaltyBoundToInterval(a3+a1*0.5-a2, 1.0, cfg_->optim.penalty_epsilon);
  _error[2] = penaltyBoundToInterval(a3+a1*0.5+a2, 1.0, cfg_->optim.penalty_epsilon);

  ROS_ASSERT_MSG(std::isfinite(a1), "EdgeAccelerationHolonomic6Goal::computeError(): a1=%f\n",a1);
  ROS_ASSERT_MSG(std::isfinite(a2), "EdgeAccelerationHolonomic6Goal::computeError(): a2=%f\n",a2);
  ROS_ASSERT_MSG(std::isfinite(a3), "EdgeAccelerationHolonomic6Goal::computeError(): a3=%f\n",a3);
}

}; // end namespace


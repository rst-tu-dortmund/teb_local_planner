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
 *********************************************************************/

#pragma once

#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{


/**
 * @class EdgeVelocityObstacleRatio
 * @brief Edge defining the cost function for keeping a minimum distance from obstacles.
 *
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point ) \cdot weight \f$. \n
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeInflatedObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
class EdgeVelocityObstacleRatio : public BaseTebMultiEdge<2, const Obstacle*>
{
public:

  /**
   * @brief Construct edge.
   */
  EdgeVelocityObstacleRatio()
  {
    // The three vertices are two poses and one time difference
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setObstacle() on EdgeVelocityObstacleRatio()");
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
    double vel = dist / deltaT->estimate();

    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction

    const double omega = angle_diff / deltaT->estimate();

    double dist_to_obstacle = cfg_->robot_model->calculateDistance(conf1->pose(), _measurement);

    double ratio;
    if (dist_to_obstacle < cfg_->obstacles.obstacle_proximity_lower_bound)
      ratio = 0;
    else if (dist_to_obstacle > cfg_->obstacles.obstacle_proximity_upper_bound)
      ratio = 1;
    else
      ratio = (dist_to_obstacle - cfg_->obstacles.obstacle_proximity_lower_bound) /
      (cfg_->obstacles.obstacle_proximity_upper_bound - cfg_->obstacles.obstacle_proximity_lower_bound);
    ratio *= cfg_->obstacles.obstacle_proximity_ratio_max_vel;

    const double max_vel_fwd = ratio * cfg_->robot.max_vel_x;
    const double max_omega = ratio * cfg_->robot.max_vel_theta;
    _error[0] = penaltyBoundToInterval(vel, max_vel_fwd, 0);
    _error[1] = penaltyBoundToInterval(omega, max_omega, 0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]) || std::isfinite(_error[1]), "EdgeVelocityObstacleRatio::computeError() _error[0]=%f , _error[1]=%f\n",_error[0],_error[1]);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    _measurement = obstacle;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end namespace

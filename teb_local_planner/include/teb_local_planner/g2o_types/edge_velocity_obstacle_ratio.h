#pragma once


#include <teb_local_planner/optimal_planner.h>


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
  EdgeVelocityObstacleRatio() :
    robot_model_(nullptr)
  {
    // The three vertices are two poses and one time difference
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeVelocityObstacleRatio()");
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

    double dist_to_obstacle = robot_model_->calculateDistance(conf1->pose(), _measurement);

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
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

protected:

  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


} // end namespace

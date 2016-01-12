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

#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_binary_edge.h"

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public g2o::BaseBinaryEdge<1, const Obstacle*, VertexPose, VertexTimeDiff>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle() : vert_idx_(0)
  {
    _vertices[0] = _vertices[1] = NULL;
  }
  
  /**
   * @brief Construct edge and specify the vertex id (neccessary for computeError).
   * @param vert_idx Index of the vertex (position in the pose sequence)
   */      
  EdgeDynamicObstacle(size_t vert_idx) : vert_idx_(vert_idx)
  {
    _vertices[0] = _vertices[1] = NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */   
  virtual ~EdgeDynamicObstacle() 
  {
    if(_vertices[0]) _vertices[0]->edges().erase(this);
    if(_vertices[1]) _vertices[1]->edges().erase(this);
  }

  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeDynamicObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    const VertexTimeDiff* dt_vertex = static_cast<const VertexTimeDiff*>(_vertices[1]);
    
    // WARNING: vert_idx_*dt is just an approximation for the total time, since we don't have a uniform dt at the moment!
    Eigen::Vector2d pred_obst_point = _measurement->getCentroid() + double(vert_idx_)*dt_vertex->estimate()*_measurement->getCentroidVelocity();
    double dist = (pred_obst_point - bandpt->position()).norm();
    /*
    // get point in x-y-t
    Eigen::Vector3d point(bandpt->estimate().coeffRef(0), bandpt->estimate().coeffRef(1), _vert_idx*dt_vertex->estimate());
    
    // calc distance of that point to the obstacle trajectory in x-y-t predicted with a constant velocity
    Eigen::Vector3d robot_point;
    robot_point.head(2) = _measurement->getCentroid();
    robot_point.coeffRef(2) = 0;
    Eigen::Vector3d robot_vel;
    robot_vel.head(2) = _measurement->getCentroidVelocity();
    robot_vel.coeffRef(2) = 1;
    double dist = 0; //calcDistancePointToLine<Eigen::Vector3d>(point, robot_point, robot_vel);   
    */ 
    
    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);	  
  }

  /**
   * @brief Compute and return error / cost value.
   * 
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
   * @return 1D Cost / error vector
   */  
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }
  
  /**
   * @brief Read values from input stream
   */   
  virtual bool read(std::istream& is)
  {
    //  is >> _measurement[0];
    return true;
  }

  /**
   * @brief Write values to an output stream
   */   
  virtual bool write(std::ostream& os) const
  {
    //  os << information()(0,0) << " Error: " << _error[0] << ", Measurement X: " << _measurement[0] << ", Measurement Y: " << _measurement[1];
    return os.good();
  }
  
  /**
   * @brief Set the vertex index (position in the pose sequence)
   * @param vert_idx Index of the vertex
   */  
  void setVertexIdx(size_t vert_idx)
  {
    vert_idx_ = vert_idx;
  }
  
  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */     
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Assign the TebConfig class for parameters.
   * @param cfg TebConfig class
   */  
  void setTebConfig(const TebConfig& cfg)
  {
    cfg_ = &cfg;
  }

protected:
  
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
  size_t vert_idx_; //!< Store vertex index (position in the pose sequence)
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    
 
    

} // end namespace

#endif

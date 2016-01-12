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
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_multi_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgeAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
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
class EdgeAcceleration : public g2o::BaseMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   */	   
  EdgeAcceleration()
  {
    this->resize(5);
    _vertices[0]=_vertices[1]=_vertices[2]=_vertices[3]=_vertices[4]=NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */  
  virtual ~EdgeAcceleration()
  {
    for(unsigned int i=0;i<5;i++)
    {
	if(_vertices[i])
	  _vertices[i]->edges().erase(this);
    }
  }
   
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    double vel1 = diff1.norm() / dt1->dt();
    double vel2 = diff2.norm() / dt2->dt();
    
    // consider directions
//     vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
//     vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta())); 
    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) ); 
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) ); 
    
    double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );
   

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
    double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
    double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
      
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
  }



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
	
  /**
   * @brief Compute and return error / cost value.
   * 
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
   * @return 2D Cost / error vector [translational acc cost, angular acc cost]^T
   */ 	
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }	
	
  /**
   * @brief Read values from input stream
   */  	
  bool read(std::istream& is)
  {
    is >> _measurement;
    is >> information()(0,0);	// TODO: fixme
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  bool write(std::ostream& os) const
  {
    os << information()(0,0) << " Error: " << _error[0] << " " << _error[1]; // TODO: fixme

    return os.good();
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
      
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};
    
    
/**
 * @class EdgeAccelerationStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
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
class EdgeAccelerationStart : public g2o::BaseMultiEdge<2, const Eigen::Vector2d*>
{
public:

  /**
   * @brief Construct edge.
   */	  
  EdgeAccelerationStart()
  {
    this->resize(3);
    _vertices[0]=_vertices[1]=_vertices[2]=NULL;
    _measurement = NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */   
  ~EdgeAccelerationStart()
  {
    for(unsigned int i=0;i<3;i++)
    {
      if(_vertices[i]) 
        _vertices[i]->edges().erase(this);
    }
  }
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff = pose2->position() - pose1->position();
    double vel1 = _measurement->coeffRef(0);
    double vel2 = diff.norm() / dt->dt();

    // consider directions
    //vel2 *= g2o::sign(diff[0]*cos(pose1->theta()) + diff[1]*sin(pose1->theta())); 
    vel2 *= fast_sigmoid( 100*(diff.x()*cos(pose1->theta()) + diff.y()*sin(pose1->theta())) ); 
    
    double acc_lin  = (vel2 - vel1) / dt->dt();
    
    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = _measurement->coeffRef(1);
    double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
    double acc_rot  = (omega2 - omega1) / dt->dt();
      
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
  }

  /**
   * @brief Compute and return error / cost value.
   * 
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
   * @return 2D Cost / error vector [translational acc cost, angular acc cost]^T
   */   
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }	
	
  /**
   * @brief Read values from input stream
   */  	
  bool read(std::istream& is)
  {
    is >> information()(0,0);	// TODO: fixme
    return true;
  }

  /**
   * @brief Write values to an output stream
   */   
  bool write(std::ostream& os) const
  {
    os << information()(0,0) << " Error: " << _error[0] << " " << _error[1]; // TODO: fixme
    return os.good();
  }
  
  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start 2D vector containing the translational and rotational velocity
   */    
  void setInitialVelocity(const Eigen::Vector2d& vel_start)
  {
    _measurement = &vel_start;
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
      
public:       
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};    
    
    
    

/**
 * @class EdgeAccelerationGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
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
class EdgeAccelerationGoal : public g2o::BaseMultiEdge<2, const Eigen::Vector2d*>
{
public:

  /**
   * @brief Construct edge.
   */  
  EdgeAccelerationGoal()
  {
    _measurement = NULL;
    this->resize(3);
    _vertices[0]=_vertices[1]=_vertices[2]=NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */    
  ~EdgeAccelerationGoal()
  {
    for(unsigned int i=0;i<3;i++)
    {
      if(_vertices[i])
	_vertices[i]->edges().erase(this);
    }
  }

  /**
   * @brief Actual cost function
   */ 
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION

    Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();    
    double vel1 = diff.norm() / dt->dt();
    double vel2 = _measurement->coeffRef(0);
    
    // consider directions
    //vel1 *= g2o::sign(diff[0]*cos(pose_pre_goal->theta()) + diff[1]*sin(pose_pre_goal->theta())); 
    vel1 *= fast_sigmoid( 100*(diff.x()*cos(pose_pre_goal->theta()) + diff.y()*sin(pose_pre_goal->theta())) ); 
    
    double acc_lin  = (vel2 - vel1) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
    double omega2 = _measurement->coeffRef(1);
    double acc_rot  = (omega2 - omega1) / dt->dt();
      
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
  }
  
  
  /**
   * @brief Compute and return error / cost value.
   * 
   * This method is called by TebOptimalPlanner::computeCurrentCost to obtain the current cost.
   * @return 2D Cost / error vector [translational acc cost, angular acc cost]^T
   */   
  ErrorVector& getError()
  {
    computeError();
    return _error;
  }  
	
	
  /**
   * @brief Read values from input stream
   */  	
  bool read(std::istream& is)
  {
    is >> information()(0,0);	// TODO: fixme
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  bool write(std::ostream& os) const
  {
    os << information()(0,0) << " Error: " << _error[0] << " " << _error[1]; // TODO: fixme

    return os.good();
  }
  
  
  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal 2D vector containing the translational and rotational velocity
   */    
  void setGoalVelocity(const Eigen::Vector2d& vel_goal)
  {
    _measurement = &vel_goal;
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

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; 
    

}; // end namespace

#endif /* EDGE_ACCELERATION_H_ */

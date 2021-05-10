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
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>


#include <iostream>

namespace teb_local_planner
{

  
/**
 * @class EdgeVelocity
 * @brief Edge defining the cost function for limiting the translational and rotational velocity.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
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
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
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
    
//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    vel *= fast_sigmoid( 100.0 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
    
    const double omega = angle_diff / deltaT->estimate();
  
    _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }

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
 * @class EdgeVelocityHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T ) \cdot weight \f$. \n
 * \e vx denotes the translational velocity w.r.t. x-axis (computed using finite differneces). \n
 * \e vy denotes the translational velocity w.r.t. y-axis (computed using finite differneces). \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational velocity w.r.t. x-axis,
 * the second one w.r.t. the y-axis and the third one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */  
class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double>
{
public:
  
  /**
   * @brief Construct edge.
   */       
  EdgeVelocityHolonomic()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  
  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocityHolonomic()");
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
    
    _error[0] = penaltyBoundToInterval(vx, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(vy, cfg_->robot.max_vel_y, 0.0); // we do not apply the penalty epsilon here, since the velocity could be close to zero
    _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]),
                   "EdgeVelocityHolonomic::computeError() _error[0]=%f _error[1]=%f _error[2]=%f\n",_error[0],_error[1],_error[2]);
  }
 
 /**
 * @class EdgeSteeringRate
 * @brief Edge defining the cost function for limiting the steering rate w.r.t. the current wheelbase parameter
 *
 * The edge depends on four vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2} \Delta T_i \f$ .
 * @remarks This edge requires the TebConfig::Robot::whelbase parameter to be set.
 * @remarks Do not forget to call setTebConfig()
 */ 
class EdgeSteeringRate : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */	      
  EdgeSteeringRate()
  {
    this->resize(5); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringRate()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* conf3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    Eigen::Vector2d delta_s1 = conf2->estimate().position() - conf1->estimate().position();
    Eigen::Vector2d delta_s2 = conf3->estimate().position() - conf2->estimate().position();
    double dist1 = delta_s1.norm();
    double dist2 = delta_s2.norm();
    double angle_diff1 = g2o::normalize_theta( conf2->theta() - conf1->theta() );
    double angle_diff2 = g2o::normalize_theta( conf3->theta() - conf2->theta() );

    double phi1, phi2;
    if (std::abs(dist1) < 1e-12)
    {
        phi1 = 0; // TODO previous phi?
        //ROS_INFO("phi 1 is zero!");
    }
    else
    {
      //dist1 *= fast_sigmoid( 100.0 * (delta_s1.x()*cos(conf1->theta()) + delta_s1.y()*sin(conf1->theta())) ); // consider direction
       //if (delta_s1.x()*cos(conf1->theta()) + delta_s1.y()*sin(conf1->theta()) < 0)
        //dist1 = -dist1;

        if (cfg_->trajectory.exact_arc_length)
            phi1 = std::atan(cfg_->robot.wheelbase / dist1 * 2.0*std::sin(angle_diff1/2.0));
        else
            phi1 = std::atan(cfg_->robot.wheelbase / dist1 * angle_diff1);

        // if we compute the sign of dist1 using the following method, the optimizer get's stuck (possibly due to non-smoothness and atan)
        // In case if we apply the sign to the angle directly, it seems to work:
        phi1 *= fast_sigmoid( 100.0 * (delta_s1.x()*cos(conf1->theta()) + delta_s1.y()*sin(conf1->theta())) ); // consider direction
    }

    if (std::abs(dist2) < 1e-12)
    {
        phi2 = phi1;
        ROS_INFO("phi 2 is phi1!");
    }
    else
    {
        //dist2 *= fast_sigmoid( 100.0 * (delta_s2.x()*cos(conf2->theta()) + delta_s2.y()*sin(conf2->theta())) ); // consider direction
        //if (delta_s2.x()*cos(conf2->theta()) + delta_s2.y()*sin(conf2->theta()) < 0)
       //   dist2 = -dist2;

        if (cfg_->trajectory.exact_arc_length)
            phi2 = std::atan(cfg_->robot.wheelbase / dist2 * 2.0*std::sin(angle_diff2/2.0));
        else
            phi2 = std::atan(cfg_->robot.wheelbase / dist2 * angle_diff2);

        // if we compute the sign of dist1 using the following method, the optimizer get's stuck (possibly due to non-smoothness and atan).
        // In case if we apply the sign to the angle directly, it seems to work:
        phi2 *= fast_sigmoid( 100.0 * (delta_s2.x()*cos(conf2->theta()) + delta_s2.y()*sin(conf2->theta())) ); // consider direction
    }

    _error[0] = penaltyBoundToInterval(g2o::normalize_theta(phi2 - phi1)*2.0 / (dt1->dt() + dt2->dt()), cfg_->robot.max_steering_rate, 0.0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRate::computeError() _error[0]\n",_error[0]);
  }

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

//! Corresponds to EdgeSteeringRate but with initial steering angle for the predecessor configuration
class EdgeSteeringRateStart : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */	      
  EdgeSteeringRateStart()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringRateStart()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d delta_s = conf2->estimate().position() - conf1->estimate().position();
    double dist = delta_s.norm();
    double angle_diff = g2o::normalize_theta( conf2->theta() - conf1->theta() );

    double phi;
    if (std::abs(dist) < 1e-12)
    {
        ROS_INFO("Start phi equals pervious phi!");
        phi = _measurement;
    }
    else
    {
       //dist *= fast_sigmoid( 100.0 * (delta_s.x()*cos(conf1->theta()) + delta_s.y()*sin(conf1->theta())) ); // consider direction
       //dist *= (double)g2o::sign( delta_s.x()*cos(conf1->theta()) + delta_s.y()*sin(conf1->theta()) ); // consider direction

        if (cfg_->trajectory.exact_arc_length)
            phi = std::atan(cfg_->robot.wheelbase / dist * 2.0*std::sin(angle_diff/2.0));
        else
            phi = std::atan(cfg_->robot.wheelbase / dist * angle_diff);

        // if we compute the sign of dist1 using the following method, the optimizer get's stuck (possibly due to non-smoothness and atan).
        // In case if we apply the sign to the angle directly, it seems to work:
        phi *= fast_sigmoid( 100.0 * (delta_s.x()*cos(conf1->theta()) + delta_s.y()*sin(conf1->theta())) ); // consider direction
    }

    _error[0] = penaltyBoundToInterval(g2o::normalize_theta(phi - _measurement) / dt->dt(), cfg_->robot.max_steering_rate, 0.0);


    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRateStart::computeError() _error[0]\n",_error[0]);
  }

  void setInitialSteeringAngle(double steering_angle)
  {
      _measurement = steering_angle;
  }

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

//! Corresponds to EdgeSteeringRate but with initial steering angle for the successor configuration
class EdgeSteeringRateGoal : public BaseTebMultiEdge<1, double>
{
public:

  /**
   * @brief Construct edge.
   */	      
  EdgeSteeringRateGoal()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }

  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeSteeringRateGoal()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    Eigen::Vector2d delta_s = conf2->estimate().position() - conf1->estimate().position();
    double dist = delta_s.norm();
    double angle_diff = g2o::normalize_theta( conf2->theta() - conf1->theta() );

    double phi;
    if (std::abs(dist) < 1e-12)
    {
      ROS_INFO("Goal phi is zero!");
        phi = 0;
    }
    else
    {
        //dist *= fast_sigmoid( 100.0 * (delta_s.x()*cos(conf1->theta()) + delta_s.y()*sin(conf1->theta())) ); // consider direction

        if (cfg_->trajectory.exact_arc_length)
            phi = std::atan(cfg_->robot.wheelbase / dist * 2.0*std::sin(angle_diff/2.0));
        else
            phi = std::atan(cfg_->robot.wheelbase / dist * angle_diff);

        // if we compute the sign of dist1 using the following method, the optimizer get's stuck (possibly due to non-smoothness and atan).
        // In case if we apply the sign to the angle directly, it seems to work:
        phi *= fast_sigmoid( 100.0 * (delta_s.x()*cos(conf1->theta()) + delta_s.y()*sin(conf1->theta())) ); // consider direction
    }

    _error[0] = penaltyBoundToInterval(g2o::normalize_theta(_measurement - phi) / dt->dt(), cfg_->robot.max_steering_rate, 0.0);


    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeSteeringRateGoal::computeError() _error[0]\n",_error[0]);
  }

  void setGoalSteeringAngle(double steering_angle)
  {
      _measurement = steering_angle;
  }

public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; 



} // end namespace

#endif

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

#ifndef EDGE_TIMEOPTIMAL_H_
#define EDGE_TIMEOPTIMAL_H_

#include <float.h>

#include <g2o/core/base_unary_edge.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include <Eigen/Core>

namespace teb_local_planner
{

  
/**
 * @class EdgeTimeOptimal
 * @brief Edge defining the cost function for minimizing transition time of the trajectory.
 * 
 * The edge depends on a single vertex \f$ \Delta T_i \f$ and minimizes: \n
 * \f$ \min \Delta T_i^2 \cdot scale \cdot weight \f$. \n
 * \e scale is determined using the penaltyEquality() function, since we experiences good convergence speeds with it. \n
 * \e weight can be set using setInformation() (something around 1.0 seems to be fine). \n
 * @see TebOptimalPlanner::AddEdgesTimeOptimal
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeTimeOptimal : public g2o::BaseUnaryEdge<1, double, VertexTimeDiff>
{
public:
		
  /**
   * @brief Construct edge.
   */
  EdgeTimeOptimal()
  {
    this->setMeasurement(0.);
    _vertices[0] = NULL;
  }
  
  /**
   * @brief Destruct edge.
   * 
   * We need to erase vertices manually, since we want to keep them even if TebOptimalPlanner::clearGraph() is called.
   * This is necessary since the vertices are managed by the Timed_Elastic_Band class.
   */
  virtual ~EdgeTimeOptimal()
  {
    if(_vertices[0]) 
      _vertices[0]->edges().erase(this); 
  }

  /**
   * @brief Actual cost function
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
    const VertexTimeDiff* timediff = static_cast<const VertexTimeDiff*>(_vertices[0]);

   _error[0] = timediff->dt();
  
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeTimeOptimal::computeError() _error[0]=%f\n",_error[0]);
  }

#ifdef USE_ANALYTIC_JACOBI
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
    const VertexTimeDiff* vi = static_cast<const VertexTimeDiff*>(_vertices[0]);
    _jacobianOplusXi( 0 , 0 ) = 1;
  }
#endif
    
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
    is >> _measurement;
    is >> information()(0,0);
    return true;
  }

  /**
   * @brief Write values to an output stream
   */
  virtual bool write(std::ostream& os) const
  {
    os << information()(0,0) << " Error: " << _error[0]; 
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

}; // end namespace

#endif /* EDGE_TIMEOPTIMAL_H_ */

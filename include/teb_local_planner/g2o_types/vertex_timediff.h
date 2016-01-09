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

#ifndef VERTEX_TIMEDIFF_H
#define VERTEX_TIMEDIFF_H


#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include "ros/console.h"

#include <Eigen/Core>

namespace teb_local_planner
{

/**
  * @class VertexTimeDiff
  * @brief This class stores and wraps a time difference \f$ \Delta T \f$ into a vertex that can be optimized via g2o
  * @see VertexPointXY
  * @see VertexOrientation
  */
class VertexTimeDiff : public g2o::BaseVertex<1, double>
{
public:

  /**
    * @brief Default constructor
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */  
  VertexTimeDiff(bool fixed = false)
  {
    setToOriginImpl();
    setFixed(fixed);
  }
  
  /**
    * @brief Construct the TimeDiff vertex with a value
    * @param dt time difference value of the vertex
    * @param fixed if \c true, this vertex is considered fixed during optimization [default: \c false]
    */  
  VertexTimeDiff(double dt, bool fixed = false)
  {
    _estimate = dt;
    setFixed(fixed);
  }

  /**
    * @brief Destructs the VertexTimeDiff
    */ 
  ~VertexTimeDiff()
  {}

  /**
    * @brief Access the timediff value of the vertex
    * @see estimate
    * @return reference to dt
    */ 
  double& dt() {return _estimate;}
  
  /**
    * @brief Access the timediff value of the vertex (read-only)
    * @see estimate
    * @return const reference to dt
    */ 
  const double& dt() const {return _estimate;}
  
  /**
    * @brief Set the underlying TimeDiff estimate \f$ \Delta T \f$ to default.
    */ 
  virtual void setToOriginImpl()
  {
    _estimate = 0.1;
  }

  /**
    * @brief Define the update increment \f$ \Delta T_{k+1} = \Delta T_k + update \f$.
    * A simple addition implements what we want.
    * @param update increment that should be added to the previous esimate
    */ 
  virtual void oplusImpl(const double* update)
  {
      _estimate += *update;
  }

  /**
    * @brief Read an estimate of \f$ \Delta T \f$ from an input stream
    * @param is input stream
    * @return always \c true
    */ 
  virtual bool read(std::istream& is)
  {
    is >> _estimate;
    return true;
  }

  /**
    * @brief Write the estimate \f$ \Delta T \f$ to an output stream
    * @param os output stream
    * @return \c true if the export was successful, otherwise \c false
    */ 
  virtual bool write(std::ostream& os) const
  {
    os << estimate();
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif

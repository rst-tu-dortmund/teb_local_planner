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

#ifndef _BASE_TEB_EDGES_H_
#define _BASE_TEB_EDGES_H_

#include <teb_local_planner/teb_config.h>

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>

#include <cmath>

namespace teb_local_planner
{

/**
 * @brief Common interface for setting/getting the TebConfig.
 *
 * Class offers a setter/getter interface used by the edges of this library.
 */
struct ConfigInterface{

  inline const TebConfig* getTebConfig() const noexcept
  {
    return cfg_;
  }

  inline void setTebConfig(const TebConfig& _cfg) noexcept
  {
    cfg_ = &_cfg;
  }

protected:
  const TebConfig* cfg_;
};

/**
 * @class BaseTebUnaryEdge
 * @brief Base edge connecting a single vertex in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebBinaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */   
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>,
                         public ConfigInterface
{
public:

  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};

/**
 * @class BaseTebBinaryEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebMultiEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */    
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>,
                          public ConfigInterface
{
public:
  
  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};


/**
 * @class BaseTebMultiEdge
 * @brief Base edge connecting two vertices in the TEB optimization problem
 * 
 * This edge defines a base edge type for the TEB optimization problem.
 * It is derived from the corresponding g2o base classes augmented with additional information for the dedicated TEB problem (e.g. config).
 * The destructor erases the edge in all attached vertices in order to allow keeping the vertices valid in subsequent g2o optimization calls.
 * Memory of edges should be freed by calling the clearEdge method of the g2o optimzier class.
 * @see BaseTebBinaryEdge, BaseTebUnaryEdge, g2o::BaseBinaryEdge, g2o::BaseUnaryEdge, g2o::BaseMultiEdge
 */    
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>,
                         public ConfigInterface
{
public:

  /**
   * @brief Read values from input stream
   */  	
  virtual bool read(std::istream& is)
  {
    // TODO generic read
    return true;
  }

  /**
   * @brief Write values to an output stream
   */    
  virtual bool write(std::ostream& os) const
  {
    // TODO generic write
    return os.good();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
};






} // end namespace

#endif

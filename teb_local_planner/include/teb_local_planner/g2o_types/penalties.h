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
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef PENALTIES_H
#define PENALTIES_H

#include <cmath>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace teb_local_planner
{

/**
 * @brief Linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundToInterval(const double& var,const double& a,const double& epsilon)
{
  if (var < -a+epsilon)
  {
    return (-var - (a - epsilon));
  }
  if (var <= a-epsilon)
  {
    return 0.;
  }
  else
  {
    return (var - (a - epsilon));
  }
}

/**
 * @brief Linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundToInterval(const double& var,const double& a, const double& b, const double& epsilon)
{
  if (var < a+epsilon)
  {
    return (-var + (a + epsilon));
  }
  if (var <= b-epsilon)
  {
    return 0.;
  }
  else
  {
    return (var - (b - epsilon));
  }
}


/**
 * @brief Linear penalty function for bounding \c var from below: \f$ a < var \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundFromBelowDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
{
  if (var >= a+epsilon)
  {
    return 0.;
  }
  else
  {
    return (-var + (a+epsilon));
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundToIntervalDerivative(const double& var,const double& a, const double& epsilon)
{
  if (var < -a+epsilon)
  {
    return -1;
  }
  if (var <= a-epsilon)
  {
    return 0.;
  }
  else
  {
    return 1;		
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundToIntervalDerivative(const double& var,const double& a, const double& b, const double& epsilon)
{
  if (var < a+epsilon)
  {
    return -1;
  }
  if (var <= b-epsilon)
  {
    return 0.;
  }
  else
  {
    return 1;		
  }
}


/**
 * @brief Derivative of the linear penalty function for bounding \c var from below: \f$ a < var \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundFromBelow
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundFromBelowDerivative(const double& var, const double& a,const double& epsilon)
{
  if (var >= a+epsilon)
  {
    return 0.;
  }
  else
  {
    return -1;
  }
}


} // namespace teb_local_planner


#endif // PENALTIES_H

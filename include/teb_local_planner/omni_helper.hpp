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

#ifndef OMNI_HELPER_H_
#define OMNI_HELPER_H_

#include <cmath>
#include <algorithm>

namespace teb_local_planner
{

// error
// a1: vel_x, a2: vel_y, a3: omega_z
inline double error_0_1(const double& a1, const double& a2, const double a3) { return std::hypot(std::hypot(a1, a2), a3); }

inline double error_1_1(const double& a1, const double& a2, const double a3) { return a3 - a1 + a2; }
inline double error_1_2(const double& a1, const double& a2, const double a3) { return a3 - a1 - a2; }
inline double error_1_3(const double& a1, const double& a2, const double a3) { return a3 + a1 - a2; }
inline double error_1_4(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2; }

inline double error_2_1(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double error_2_2(const double& a1, const double& a2, const double a3) { return a3 - a1; }
inline double error_2_3(const double& a1, const double& a2, const double a3) { return a3 - a2; }
inline double error_2_4(const double& a1, const double& a2, const double a3) { return a3 + a1; }

inline double error_3_1(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double error_3_2(const double& a1, const double& a2, const double a3) { return a3 - a1 - a2 * 0.5; }
inline double error_3_3(const double& a1, const double& a2, const double a3) { return a3 + a1 - a2 * 0.5; }

inline double error_4_1(const double& a1, const double& a2, const double a3) { return a3 + a1; }
inline double error_4_2(const double& a1, const double& a2, const double a3) { return a3 - a1 * 0.5 + a2; }
inline double error_4_3(const double& a1, const double& a2, const double a3) { return a3 - a1 * 0.5 - a2; }

inline double error_5_1(const double& a1, const double& a2, const double a3) { return a3 - a2; }
inline double error_5_2(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2 * 0.5; }
inline double error_5_3(const double& a1, const double& a2, const double a3) { return a3 - a1 + a2 * 0.5; }

inline double error_6_1(const double& a1, const double& a2, const double a3) { return a3 - a1; }
inline double error_6_2(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 - a2; }
inline double error_6_3(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 + a2; }


// weight
// a1: vel_x, a2: vel_y, a3: omega_z
inline double weight_0_1(const double& a1, const double& a2, const double a3) { return std::hypot(std::hypot(a1, a2), a3); }

inline double weight_1_1(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2; }
inline double weight_1_2(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2; }
inline double weight_1_3(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2; }
inline double weight_1_4(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2; }

inline double weight_2_1(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double weight_2_2(const double& a1, const double& a2, const double a3) { return a3 + a1; }
inline double weight_2_3(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double weight_2_4(const double& a1, const double& a2, const double a3) { return a3 + a1; }

inline double weight_3_1(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double weight_3_2(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2 * 0.5; }
inline double weight_3_3(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2 * 0.5; }

inline double weight_4_1(const double& a1, const double& a2, const double a3) { return a3 + a1; }
inline double weight_4_2(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 + a2; }
inline double weight_4_3(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 + a2; }

inline double weight_5_1(const double& a1, const double& a2, const double a3) { return a3 + a2; }
inline double weight_5_2(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2 * 0.5; }
inline double weight_5_3(const double& a1, const double& a2, const double a3) { return a3 + a1 + a2 * 0.5; }

inline double weight_6_1(const double& a1, const double& a2, const double a3) { return a3 + a1; }
inline double weight_6_2(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 + a2; }
inline double weight_6_3(const double& a1, const double& a2, const double a3) { return a3 + a1 * 0.5 + a2; }

// limit
// a1: vel_x, a2: vel_y, a3: omega_z
inline double limit_0(const double& a1, const double& a2, const double a3) { return std::hypot(std::hypot(a1, a2), a3); }
inline double limit_1(const double& a1, const double& a2, const double a3) { return std::fabs(a1) + std::fabs(a2) + std::fabs(a3); }
inline double limit_2(const double& a1, const double& a2, const double a3) { return std::max(std::fabs(a1), std::fabs(a2)) + std::fabs(a3); }
inline double limit_3(const double& a1, const double& a2, const double a3) { return std::max(std::fabs(a3 + a2), std::max(std::fabs(a3 - a1 - a2 * 0.5), std::fabs(a3 + a1 - a2 * 0.5))); }
inline double limit_4(const double& a1, const double& a2, const double a3) { return std::max(std::fabs(a3 + a1), std::max(std::fabs(a3 - a1 * 0.5 + a2), std::fabs(a3 - a1 * 0.5 - a2))); }
inline double limit_5(const double& a1, const double& a2, const double a3) { return std::max(std::fabs(a3 - a2), std::max(std::fabs(a3 + a1 + a2 * 0.5), std::fabs(a3 - a1 + a2 * 0.5))); }
inline double limit_6(const double& a1, const double& a2, const double a3) { return std::max(std::fabs(a3 - a1), std::max(std::fabs(a3 + a1 * 0.5 - a2), std::fabs(a3 + a1 * 0.5 + a2))); }

} // namespace teb_local_planner

#endif

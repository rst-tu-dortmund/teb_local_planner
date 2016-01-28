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

#ifndef MISC_H
#define MISC_H

#include <Eigen/Core>
#include <boost/utility.hpp>
#include <boost/type_traits.hpp>


namespace teb_local_planner
{

#define SMALL_NUM 0.00000001


// The Distance Calculations are mainly copied from http://geomalgorithms.com/a07-_distance.html
// Copyright 2001 softSurfer, 2012 Dan Sunday
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

inline double calc_distance_line_to_line_3d(const Eigen::Ref<const Eigen::Vector3d>& x1, Eigen::Ref<const Eigen::Vector3d>& u,
					    const Eigen::Ref<const Eigen::Vector3d>& x2, Eigen::Ref<const Eigen::Vector3d>& v)
{
  Eigen::Vector3d   w = x2 - x1;
  double a = u.squaredNorm();         // dot(u,u) always >= 0
  double b = u.dot(v);
  double c = v.squaredNorm();         // dot(v,v) always >= 0
  double d = u.dot(w);
  double e = v.dot(w);
  double D = a*c - b*b;        // always >= 0
  double sc, tc;

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) {          // the lines are almost parallel
    sc = 0.0;
    tc = (b>c ? d/b : e/c);    // use the largest denominator
  }
  else {
    sc = (b*e - c*d) / D;
    tc = (a*e - b*d) / D;
  }

  // get the difference of the two closest points
  Eigen::Vector3d dP = w + (sc * u) - (tc * v);  // =  L1(sc) - L2(tc)

  return dP.norm();   // return the closest distance
}





inline double calc_distance_segment_to_segment3D(const Eigen::Ref<const Eigen::Vector3d>& line1_start, Eigen::Ref<const Eigen::Vector3d>& line1_end,
						 const Eigen::Ref<const Eigen::Vector3d>& line2_start, Eigen::Ref<const Eigen::Vector3d>& line2_end)
{
  Eigen::Vector3d u = line1_end - line1_start;
  Eigen::Vector3d v = line2_end - line2_start;
  Eigen::Vector3d w = line2_start - line1_start;
  double a = u.squaredNorm();         // dot(u,u) always >= 0
  double b = u.dot(v);
  double c = v.squaredNorm();         // dot(v,v) always >= 0
  double d = u.dot(w);
  double e = v.dot(w);
  double D = a*c - b*b;        // always >= 0
  double sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
  double tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) 
  { // the lines are almost parallel
    sN = 0.0;         // force using point P0 on segment S1
    sD = 1.0;         // to prevent possible division by 0.0 later
    tN = e;
    tD = c;
  }
  else 
  {       // get the closest points on the infinite lines
    sN = (b*e - c*d);
    tN = (a*e - b*d);
    if (sN < 0.0)
    {        // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    }
    else if (sN > sD)
    {  // sc > 1  => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) 
  {   // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0)
      sN = 0.0;
    else if (-d > a)
      sN = sD;
    else 
    {
      sN = -d;
      sD = a;
    }
  }
  else if (tN > tD)
  {      // tc > 1  => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0)
      sN = 0;
    else if ((-d + b) > a)
      sN = sD;
    else
    {
      sN = (-d +  b);
      sD = a;
    }
  }
  // finally do the division to get sc and tc
  sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  Eigen::Vector3d dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

  return dP.norm();   // return the closest distance
}




template <typename VectorType>
double calc_closest_point_to_approach_time(const VectorType& x1, const VectorType& vel1, const VectorType& x2, const VectorType& vel2)
{
  VectorType dv = vel1 - vel2;

  double dv2 = dv.squaredNorm(); // dot(v,v)
  if (dv2 < SMALL_NUM)      // the  tracks are almost parallel
    return 0.0;             // any time is ok.  Use time 0.

  VectorType w0 = x1 - x2;
  double cpatime = -w0.dot(dv) / dv2;

  return cpatime;             // time of CPA
}

template <typename VectorType>
double calc_closest_point_to_approach_distance(const VectorType& x1, const VectorType& vel1, const VectorType& x2, const VectorType& vel2, double bound_cpa_time = 0)
{
  double ctime = calc_closest_point_to_approach_time<VectorType>(x1, vel1, x2, vel2);
  if (bound_cpa_time!=0 && ctime > bound_cpa_time) ctime = bound_cpa_time;
  VectorType P1 = x1 + (ctime * vel1);
  VectorType P2 = x2 + (ctime * vel2);

  return (P2-P1).norm(); // distance at CPA
}



// dist_Point_to_Line(): get the distance of a point to a line
//     Input:  a Point P and a Line L (in any dimension)
//     Return: the shortest distance from P to L
template <typename VectorType>
double calc_distance_point_to_line( const VectorType& point, const VectorType& line_base, const VectorType& line_dir)
{
  VectorType w = point - line_base;

  double c1 = w.dot(line_dir);
  double c2 = line_dir.dot(line_dir);
  double b = c1 / c2;

  VectorType Pb = line_base + b * line_dir;
  return (point-Pb).norm();
}
//===================================================================


// dist_Point_to_Segment(): get the distance of a point to a segment
//     Input:  a Point P and a Segment S (in any dimension)
//     Return: the shortest distance from P to S
template <typename VectorType>
double calc_distance_point_to_segment( const VectorType& point, const VectorType& line_start, const VectorType& line_end)
{
  VectorType v = line_end - line_start;
  VectorType w = point - line_start;

  double c1 = w.dot(v);
  if ( c1 <= 0 )
    return w.norm();

  double c2 = v.dot(v);
  if ( c2 <= c1 )
    return (point-line_end).norm();

  double b = c1 / c2;
  VectorType Pb = line_start + b * v;
  return (point-Pb).norm();
}


/** 
 * @brief Check whether two variables (double) are close to each other
 * @param a the first value to compare
 * @param b the second value to compare
 * @param epsilon precision threshold
 * @return \c true if |a-b| < epsilon, false otherwise
 */
inline bool is_close(double a, double b, double epsilon = 1e-4) 
{ 
  return std::fabs(a - b) < epsilon; 
}

/** 
 * @brief Return the average angle of an arbitrary number of given angles [rad]
 * @param angles vector containing all angles
 * @return average / mean angle, that is normalized to [-pi, pi]
 */
inline double average_angles(const std::vector<double>& angles)
{
  double x=0, y=0;
  for (std::vector<double>::const_iterator it = angles.begin(); it!=angles.end(); ++it)
  {
      x += cos(*it);
      y += sin(*it);
  }
  if(x == 0 && y == 0)
      return 0;
  else
      return std::atan2(y, x);
}

/** @brief Small helper function: check if |a|<|b| */
inline bool smaller_than_abs(double i, double j) {return std::fabs(i)<std::fabs(j);}


/**
 * @brief Calculate a fast approximation of a sigmoid function
 * @details The following function is implemented: \f$ x / (1 + |x|) \f$
 * @param x the argument of the function
*/
inline double fast_sigmoid(double x)
{
  return x / (1 + fabs(x));
}


/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T& val).
 * @param ptr pointer of type T
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T* ptr) {return *ptr;}

/** 
 * @brief Helper function that returns the const reference to a value defined by either its raw pointer type or const reference.
 * 
 * Return a constant reference for boths input variants (pointer or reference).
 * @remarks Makes only sense in combination with the overload getConstReference(const T* val).
 * @param val
 * @param dummy SFINAE helper variable
 * @tparam T arbitrary type 
 * @return  If \c T is a pointer, return const *T (leading to const T&), otherwise const T& with out pointer-to-ref conversion
 */
template<typename T>
inline const T& get_const_reference(const T& val, typename boost::disable_if<boost::is_pointer<T> >::type* dummy = 0) {return val;}

} // namespace teb_local_planner

#endif /* MISC_H */

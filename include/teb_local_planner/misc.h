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

#include <builtin_interfaces/msg/duration.hpp>

#include <Eigen/Core>

#include <exception>
#include <type_traits>

#include <rclcpp/logging.hpp>

namespace teb_local_planner
{
#define SMALL_NUM 0.00000001

//! Symbols for left/none/right rotations      
enum class RotType { left, none, right };

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
 * @brief Calculate Euclidean distance between two 2D point datatypes
 * @param point1 object containing fields x and y
 * @param point2 object containing fields x and y
 * @return Euclidean distance: ||point2-point1||
*/
template <typename P1, typename P2>
inline double distance_points2d(const P1& point1, const P2& point2)
{
  return std::sqrt( std::pow(point2.x-point1.x,2) + std::pow(point2.y-point1.y,2) );
}


/**
 * @brief Calculate the 2d cross product (returns length of the resulting vector along the z-axis in 3d)
 * @param v1 object containing public methods x() and y()
 * @param v2 object containing fields x() and y()
 * @return magnitude that would result in the 3D case (along the z-axis)
*/
template <typename V1, typename V2>
inline double cross2d(const V1& v1, const V2& v2)
{
     return v1.x()*v2.y() - v2.x()*v1.y();
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
inline const T& get_const_reference(const T& val, typename std::enable_if_t<!std::is_pointer<T>::value, T>* dummy = nullptr) {return val;}

inline builtin_interfaces::msg::Duration durationFromSec(double t_sec)
{
  int32_t sec;
  uint32_t nsec;
  sec = static_cast<int32_t>(floor(t_sec));
  nsec = static_cast<int32_t>(std::round((t_sec - sec) * 1e9));
  // avoid rounding errors
  sec += (nsec / 1000000000l);
  nsec %= 1000000000l;

  builtin_interfaces::msg::Duration duration;
  duration.sec = sec;
  duration.nanosec = nsec;
  return duration;
}

struct TebAssertionFailureException : public std::runtime_error
{
    TebAssertionFailureException(const std::string &msg)
        : std::runtime_error(msg)
    {
        RCLCPP_ERROR(rclcpp::get_logger("teb_local_planner"), msg.c_str());
    }
};

#define TEB_ASSERT_MSG_IMPL(...) \
    { \
        char arg_string[1024]; \
        std::sprintf(arg_string, __VA_ARGS__); \
        const std::string msg(arg_string); \
        throw TebAssertionFailureException(msg); \
    }

template<typename T, typename ...ARGS, typename std::enable_if_t<std::is_arithmetic<T>::value>* = nullptr>
void teb_assert_msg_impl(const T expression, ARGS ...args) {
    if(expression == 0) {
        char arg_string[1024];
        std::sprintf(arg_string, args..., "");
        const std::string msg(arg_string);
        throw TebAssertionFailureException(msg);
    }
}

template<typename T, typename ...ARGS, typename std::enable_if_t<std::is_pointer<T>::value>* = nullptr>
void teb_assert_msg_impl(const T expression, ARGS ...args) {
    if(expression == nullptr) {
        char arg_string[1024];
        std::sprintf(arg_string, args..., "");
        const std::string msg(arg_string);
        throw TebAssertionFailureException(msg);
    }
}

#define TEB_ASSERT_MSG(expression, ...) teb_assert_msg_impl(expression, __VA_ARGS__)

} // namespace teb_local_planner

#endif /* MISC_H */

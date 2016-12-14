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

#include <teb_local_planner/recovery_behaviors.h>
#include <ros/ros.h>
#include <limits>
#include <functional>
#include <numeric>
#include <g2o/stuff/misc.h>

namespace teb_local_planner
{

// ============== FailureDetector Implementation ===================

void FailureDetector::update(const geometry_msgs::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps)
{
    if (buffer_.capacity() == 0)
        return;
    
    VelMeasurement measurement;
    measurement.v = twist.linear.x; // just consider linear velocity in x-direction in the robot frame for now
    measurement.omega = twist.angular.z;
    
    if (measurement.v > 0 && v_max>0)
        measurement.v /= v_max;
    else if (measurement.v < 0 && v_backwards_max > 0)
        measurement.v /= v_backwards_max;
    
    if (omega_max > 0)
        measurement.omega /= omega_max;
    
    buffer_.push_back(measurement);
    
    // immediately compute new state
    detect(v_eps, omega_eps);
}

void FailureDetector::clear()
{
    buffer_.clear();
    oscillating_ = false;
}

bool FailureDetector::isOscillating() const
{
    return oscillating_;
}

bool FailureDetector::detect(double v_eps, double omega_eps)
{
    oscillating_ = false;
    
    if (buffer_.size() < buffer_.capacity()/2) // we start detecting only as soon as we have the buffer filled at least half
        return false;

    double n = (double)buffer_.size();
            
    // compute mean for v and omega
    double v_mean=0;
    double omega_mean=0;
    int omega_zero_crossings = 0;
    for (int i=0; i < n; ++i)
    {
        v_mean += buffer_[i].v;
        omega_mean += buffer_[i].omega;
        if ( i>0 && g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i-1].omega) )
            ++omega_zero_crossings;
    }
    v_mean /= n;
    omega_mean /= n;

    if (std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps && omega_zero_crossings>1 ) 
    {
        oscillating_ = true;
    }
//     ROS_INFO_STREAM("v: " << std::abs(v_mean) << ", omega: " << std::abs(omega_mean) << ", zero crossings: " << omega_zero_crossings);
    return oscillating_;
}
    
    

} // namespace teb_local_planner

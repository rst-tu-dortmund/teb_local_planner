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

#ifndef RECOVERY_BEHAVIORS_H__
#define RECOVERY_BEHAVIORS_H__


#include <boost/circular_buffer.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace teb_local_planner
{


/**
 * @class FailureDetector
 * @brief This class implements methods in order to detect if the robot got stucked or is oscillating
 * 
 * The StuckDetector analyzes the last N commanded velocities in order to detect whether the robot
 * might got stucked or oscillates between left/right/forward/backwards motions.
 */  
class FailureDetector
{
public:

  /**
   * @brief Default constructor
   */
  FailureDetector() {}  
  
  /**
   * @brief destructor.
   */
  ~FailureDetector() {}
  
  /**
   * @brief Set buffer length (measurement history)
   * @param length number of measurements to be kept
   */
  void setBufferLength(int length) {buffer_.set_capacity(length);}
  
  /**
   * @brief Add a new twist measurement to the internal buffer and compute a new decision
   * @param twist geometry_msgs::Twist velocity information
   * @param v_max maximum forward translational velocity
   * @param v_backwards_max maximum backward translational velocity
   * @param omega_max maximum angular velocity 
   * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
   * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
   */
  void update(const geometry_msgs::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps);
  
  /**
   * @brief Check if the robot got stucked
   * 
   * This call does not compute the actual decision,
   * since it is computed within each update() invocation.
   * @return true if the robot got stucked, false otherwise.
   */
  bool isOscillating() const;
  
  /**
   * @brief Clear the current internal state
   * 
   * This call also resets the internal buffer
   */
  void clear();
       
protected:
    
    /** Variables to be monitored */
    struct VelMeasurement
    {
        double v = 0;
        double omega = 0;
    };
    
    /**
     * @brief Detect if the robot got stucked based on the current buffer content
     * 
     * Afterwards the status might be checked using gotStucked();
     * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
     * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
     * @return true if the robot got stucked, false otherwise
     */
    bool detect(double v_eps, double omega_eps);
  
private:
    
    boost::circular_buffer<VelMeasurement> buffer_; //!< Circular buffer to store the last measurements @see setBufferLength
    bool oscillating_ = false; //!< Current state: true if robot is oscillating
                
};


} // namespace teb_local_planner

#endif /* RECOVERY_BEHAVIORS_H__ */

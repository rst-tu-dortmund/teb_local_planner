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

#include <teb_local_planner/homotopy_class_planner.h>

namespace teb_local_planner
{
  
  
template<typename BidirIter, typename Fun>
std::complex<long double> HomotopyClassPlanner::calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles, double prescaler)
{
    if (obstacles->empty()) 
      return std::complex<double>(0,0);
   
   
    ROS_ASSERT_MSG(prescaler>0.1 && prescaler<=1, "Only a prescaler on the interval (0.1,1] ist allowed.");
    
    // guess values for f0
    // paper proposes a+b=N-1 && |a-b|<=1, 1...N obstacles
    int m = obstacles->size()-1;
    
    int a = (int) std::ceil(double(m)/2.0);
    int b = m-a;
    
    std::advance(path_end, -1); // reduce path_end by 1 (since we check line segments between those path points
    
    typedef std::complex<long double> cplx;
    // guess map size (only a really really coarse guess is required
    // use distance from start to goal as distance to each direction
    // TODO: one could move the map determination outside this function, since it remains constant for the whole planning interval
    cplx start = fun_cplx_point(*path_start);
    cplx end = fun_cplx_point(*path_end); // path_end points to the last point now after calling std::advance before
    cplx delta = end-start;
    cplx normal(-delta.imag(), delta.real());
    cplx map_bottom_left;
    cplx map_top_right;
    if (std::abs(delta) < 3.0)
    { // set minimum bound on distance (we do not want to have numerical instabilities) and 3.0 performs fine...
      map_bottom_left = start + cplx(0, -3);
      map_top_right = start + cplx(3, 3);
    }
    else
    {
      map_bottom_left = start - normal;
      map_top_right = start + delta + normal;
    }
    
    cplx H = 0;
    std::vector<double> imag_proposals(5);
     
    // iterate path
    while(path_start != path_end)
    {
      cplx z1 = fun_cplx_point(*path_start);
      cplx z2 = fun_cplx_point(*boost::next(path_start));

      for (unsigned int l=0; l<obstacles->size(); ++l) // iterate all obstacles
      {
        cplx obst_l = obstacles->at(l)->getCentroidCplx();
        //cplx f0 = (long double) prescaler * std::pow(obst_l-map_bottom_left,a) * std::pow(obst_l-map_top_right,b);
        cplx f0 = (long double) prescaler * (long double)a*(obst_l-map_bottom_left) * (long double)b*(obst_l-map_top_right);
        
        // denum contains product with all obstacles exepct j==l
        cplx Al = f0;
        for (unsigned int j=0; j<obstacles->size(); ++j)
        {
          if (j==l) 
              continue;
          cplx obst_j = obstacles->at(j)->getCentroidCplx();
          cplx diff = obst_l - obst_j;
          //if (diff.real()!=0 || diff.imag()!=0)
          if (std::abs(diff)<0.05) // skip really close obstacles
              Al /= diff;
          else
            continue;
        }
        // compute log value
        double diff2 = std::abs(z2-obst_l);
        double diff1 = std::abs(z1-obst_l);
        if (diff2 == 0 || diff1 == 0)
          continue;
        double log_real = std::log(diff2)-std::log(diff1);
        // complex ln has more than one solution -> choose minimum abs angle -> paper
        double arg_diff = std::arg(z2-obst_l)-std::arg(z1-obst_l);
        imag_proposals.at(0) = arg_diff;
        imag_proposals.at(1) = arg_diff+2*M_PI;
        imag_proposals.at(2) = arg_diff-2*M_PI;
        imag_proposals.at(3) = arg_diff+4*M_PI;
        imag_proposals.at(4) = arg_diff-4*M_PI;
        double log_imag = *std::min_element(imag_proposals.begin(),imag_proposals.end(),smaller_than_abs);
        cplx log_value(log_real,log_imag);
        //cplx log_value = std::log(z2-obst_l)-std::log(z1-obst_l); // the principal solution doesn't seem to work
        H += Al*log_value;  
      }
      ++path_start;
    }
    return H;
}


template<typename BidirIter, typename Fun>
void HomotopyClassPlanner::addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position,
                                            double start_orientation, double goal_orientation, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  tebs_.push_back( TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_) ) );
  tebs_.back()->teb().initTEBtoGoal(path_start, path_end, fun_position, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, 
                                    cfg_->robot.acc_lim_x, cfg_->robot.acc_lim_theta, start_orientation, goal_orientation, cfg_->trajectory.min_samples);
  if (start_velocity)
    tebs_.back()->setVelocityStart(*start_velocity);
}
  
} // namespace teb_local_planner


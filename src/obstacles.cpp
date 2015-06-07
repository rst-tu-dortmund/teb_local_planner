/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
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

#include <teb_local_planner/obstacles.h>
// #include <teb_local_planner/misc.h>

namespace teb_local_planner
{

Eigen::Vector2d Obstacle::ClosestPointOnLineSegment(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end)
{
  Eigen::Vector2d diff = line_end - line_start;

  double u = ((point.x() - line_start.x()) * diff.x() + (point.y() - line_start.y())*diff.y()) / diff.squaredNorm();
  
  if (u < 0) return line_start;
  else if (u > 1) return line_end;
  
  return line_start + u*diff;
}


double Obstacle::DistanceFromLineSegment(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end)
{
  return  (point - ClosestPointOnLineSegment(point, line_start, line_end)).norm(); 
}


bool Obstacle::CheckLineSegmentsIntersection(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end, 
					     const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end, Eigen::Vector2d* intersection)
{
  // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
  double s_numer, t_numer, denom, t;
  Eigen::Vector2d line1 = line1_end - line1_start;
  Eigen::Vector2d line2 = line2_end - line2_start;
  
  denom = line1.x() * line2.y() - line2.x() * line1.y();
  if (denom == 0) return false; // Collinear
  bool denomPositive = denom > 0;

  Eigen::Vector2d aux = line1_start - line2_start;
  
  s_numer = line1.x() * aux.y() - line1.y() * aux.x();
  if ((s_numer < 0) == denomPositive)  return false; // No collision

  t_numer = line2.x() * aux.y() - line2.y() * aux.x();
  if ((t_numer < 0) == denomPositive)  return false; // No collision

  if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive)) return false; // No collision
  
  // Otherwise collision detected
  t = t_numer / denom;
  if (intersection)
  {
    *intersection = line1_start + t * line1;
  }

  return true;
}



void PolygonObstacle::calcCentroid()
{
  centroid_.setZero();
  
  assert(!vertices_.empty());
  
  // if polygon is a line:
  if (noVertices()==2)
  {
    centroid_ = 0.5*(vertices_.front() + vertices_.back());
    return;
  }
  // otherwise:
  
  // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
  double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
  for (unsigned int i=0; i<noVertices()-1; ++i)
  {
    A += vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1);
  }
  A += vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1);
  A *= 0.5;
  
  for (unsigned int i=0; i<noVertices()-1; ++i)
  {
    double aux = (vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1));
    centroid_ +=  ( vertices_.at(i) + vertices_.at(i+1) )*aux;
  }
  double aux = (vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1));
  centroid_ +=  ( vertices_.at(noVertices()-1) + vertices_.at(0) )*aux;
  centroid_ /= (6*A);	
  
}


double PolygonObstacle::getMinimumDistance(const Eigen::Vector2d& position) const
{
  double dist = HUGE_VAL;
  
  assert(!vertices_.empty());
	  
  // check each polygon edge
  for (unsigned int i=0; i<vertices_.size()-1; ++i)
  {
      double new_dist = DistanceFromLineSegment(position, vertices_.at(i), vertices_.at(i+1));
//       double new_dist = calc_distance_point_to_segment( position,  vertices_.at(i), vertices_.at(i+1));
      if (new_dist < dist)
	dist = new_dist;
  }

  if (noVertices()>2) // if not a line close polygon
  {
    double new_dist = DistanceFromLineSegment(position, vertices_.at(vertices_.size()-1), vertices_.at(0)); // check last edge
    if (new_dist < dist)
      return new_dist;
  }
  
  return dist;
}


Eigen::Vector2d PolygonObstacle::getMinimumDistanceVec(const Eigen::Vector2d& position) const
{
  double dist = HUGE_VAL;
  
  assert(!vertices_.empty());
  
  Eigen::Vector2d diff;
      
  // check each polygon edge
  for (unsigned int i=0; i<vertices_.size()-1; ++i)
  {
    Eigen::Vector2d new_diff = position - ClosestPointOnLineSegment(position, vertices_.at(i), vertices_.at(i+1));
    double new_dist = diff.norm();
    if (new_dist < dist)
    {
      dist = new_dist;
      diff = new_diff;
    }
  }
    
  if (noVertices()>2) // if not a line close polygon
  {
    Eigen::Vector2d new_diff = position - ClosestPointOnLineSegment(position, vertices_.at(vertices_.size()-1), vertices_.at(0)); // check last edge
    if (new_diff.norm() < dist) return new_diff;
  }
  return diff;
}



bool PolygonObstacle::checkLineIntersection(const Eigen::Vector2d& line_start, const Eigen::Vector2d& line_end, double min_dist) const
{
  // Simple strategy, check all edge-line intersections until an intersection is found...
  // check each polygon edge
  for (unsigned int i=0; i<vertices_.size()-1; ++i)
  {
    if ( CheckLineSegmentsIntersection(line_start, line_end, vertices_.at(i), vertices_.at(i+1)) ) 
      return true;
  }
  if (noVertices()==2) // if polygon is a line
    return false;
  
  return CheckLineSegmentsIntersection(line_start, line_end, vertices_.at(vertices_.size()-1), vertices_.at(0)); //otherwise close polygon
}


} // namespace teb_local_planner
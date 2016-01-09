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

#include <teb_local_planner/obstacles.h>
#include <ros/console.h>
// #include <teb_local_planner/misc.h>

namespace teb_local_planner
{

Eigen::Vector2d Obstacle::ClosestPointOnLineSegment(const Eigen::Ref<const Eigen::Vector2d>& point, const Eigen::Ref<const Eigen::Vector2d>& line_start, const Eigen::Ref<const Eigen::Vector2d>& line_end)
{
  Eigen::Vector2d diff = line_end - line_start;
  double sq_norm = diff.squaredNorm();
  
  if (sq_norm == 0)
    return line_start;

  double u = ((point.x() - line_start.x()) * diff.x() + (point.y() - line_start.y())*diff.y()) / sq_norm;
  
  if (u <= 0) return line_start;
  else if (u >= 1) return line_end;
  
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

double Obstacle::DistanceSegmentToSegment2d(const Eigen::Ref<const Eigen::Vector2d>& line1_start, const Eigen::Ref<const Eigen::Vector2d>& line1_end, 
                  const Eigen::Ref<const Eigen::Vector2d>& line2_start, const Eigen::Ref<const Eigen::Vector2d>& line2_end)
{
  // TODO more efficient implementation
  
  // check if segments intersect
  if (CheckLineSegmentsIntersection(line1_start, line1_end, line2_start, line2_end))
    return 0;
  
  // check all 4 combinations
  std::array<double,4> distances;
  
  distances[0] = DistanceFromLineSegment(line1_start, line2_start, line2_end);
  distances[1] = DistanceFromLineSegment(line1_end, line2_start, line2_end);
  distances[2] = DistanceFromLineSegment(line2_start, line1_start, line1_end);
  distances[3] = DistanceFromLineSegment(line2_end, line1_start, line1_end);
  
  return *std::min_element(distances.begin(), distances.end());
}

void PolygonObstacle::fixPolygonClosure()
{
  if (vertices_.size()<2)
    return;
  
  if (vertices_.front().isApprox(vertices_.back()))
    vertices_.pop_back();
}

void PolygonObstacle::calcCentroid()
{
  if (vertices_.empty())
  {
    centroid_.setConstant(NAN);
    ROS_WARN("PolygonObstacle::calcCentroid(): number of vertices is empty. the resulting centroid is a vector of NANs.");
    return;
  }
  
  // if polygon is a point
  if (noVertices()==1)
  {
    centroid_ = vertices_.front();
    return;
  }
  
  // if polygon is a line:
  if (noVertices()==2)
  {
    centroid_ = 0.5*(vertices_.front() + vertices_.back());
    return;
  }
  
  // otherwise:
  
  centroid_.setZero();
    
  // calculate centroid (see wikipedia http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Polygon)
  double A = 0;  // A = 0.5 * sum_0_n-1 (x_i * y_{i+1} - x_{i+1} * y_i)
  for (int i=0; i<(int)noVertices()-1; ++i)
  {
    A += vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1);
  }
  A += vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1);
  A *= 0.5;
  
  if (A!=0)
  {
    for (int i=0; i<(int)noVertices()-1; ++i)
    {
      double aux = (vertices_.at(i).coeffRef(0) * vertices_.at(i+1).coeffRef(1) - vertices_.at(i+1).coeffRef(0) * vertices_.at(i).coeffRef(1));
      centroid_ +=  ( vertices_.at(i) + vertices_.at(i+1) )*aux;
    }
    double aux = (vertices_.at(noVertices()-1).coeffRef(0) * vertices_.at(0).coeffRef(1) - vertices_.at(0).coeffRef(0) * vertices_.at(noVertices()-1).coeffRef(1));
    centroid_ +=  ( vertices_.at(noVertices()-1) + vertices_.at(0) )*aux;
    centroid_ /= (6*A);	
  }
  else // A == 0 -> all points are placed on a 'perfect' line
  {
    // seach for the two outer points of the line with the maximum distance inbetween
    int i_cand = 0;
    int j_cand = 0;
    double min_dist = HUGE_VAL;
    for (int i=0; i<(int)noVertices(); ++i)
    {
      for (int j=i+1; j<(int)noVertices(); ++j) // start with j=i+1
      {
        double dist = (vertices_[j] - vertices_[i]).norm();
        if (dist < min_dist)
        {
          min_dist = dist;
          i_cand = i;
          j_cand = j;
        }
      }
    }
    // calc centroid of that line
    centroid_ = 0.5*(vertices_[i_cand] + vertices_[j_cand]);
  }
}


double PolygonObstacle::getMinimumDistance(const Eigen::Vector2d& position) const
{
  double dist = HUGE_VAL;
  
  assert(!vertices_.empty());
  
  // the polygon is a point
  if (noVertices() == 1)
  {
    return (position - vertices_.front()).norm();
  }
	  
  // check each polygon edge
  for (int i=0; i<(int)vertices_.size()-1; ++i)
  {
      double new_dist = DistanceFromLineSegment(position, vertices_.at(i), vertices_.at(i+1));
//       double new_dist = calc_distance_point_to_segment( position,  vertices_.at(i), vertices_.at(i+1));
      if (new_dist < dist)
        dist = new_dist;
  }

  if (noVertices()>2) // if not a line close polygon
  {
    double new_dist = DistanceFromLineSegment(position, vertices_.back(), vertices_.front()); // check last edge
    if (new_dist < dist)
      return new_dist;
  }
  
  return dist;
}



Eigen::Vector2d PolygonObstacle::getClosestPoint(const Eigen::Vector2d& position) const
{
  assert(!vertices_.empty());
  
  // the polygon is a point
  if (noVertices() == 1)
  {
    return vertices_.front();
  }
  
  Eigen::Vector2d closest_pt, new_pt;
  double dist = HUGE_VAL;
  
  // check each polygon edge
  for (int i=0; i<(int)vertices_.size()-1; ++i)
  {
    new_pt = ClosestPointOnLineSegment(position, vertices_.at(i), vertices_.at(i+1));
    double new_dist = (new_pt-position).norm();
    if (new_dist < dist)
    {
      dist = new_dist;
      closest_pt = new_pt;
    }
  }
  if (noVertices()>2) // if not a line close polygon
  {
    new_pt = ClosestPointOnLineSegment(position, vertices_.back(), vertices_.front());
    double new_dist = (new_pt-position).norm();
    if (new_dist < dist)
      return new_pt;
  }
  
  return closest_pt;
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
  
  return CheckLineSegmentsIntersection(line_start, line_end, vertices_.back(), vertices_.front()); //otherwise close polygon
}



// implements toPolygonMsg() of the base class
void PolygonObstacle::toPolygonMsg(geometry_msgs::Polygon& polygon)
{
  polygon.points.resize(vertices_.size());
  for (std::size_t i=0; i<vertices_.size(); ++i)
  {
    polygon.points[i].x = vertices_[i].x();
    polygon.points[i].y = vertices_[i].y();
    polygon.points[i].z = 0;
  }
}








} // namespace teb_local_planner

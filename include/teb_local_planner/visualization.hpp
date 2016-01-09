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

#include <teb_local_planner/visualization.h>
#include <boost/utility.hpp>


namespace teb_local_planner
{
 

template <typename GraphType>
void TebVisualization::publishGraph(const GraphType& graph, const std::string& ns_prefix)
{	 
  if ( printErrorWhenNotInitialized() )
    return;
  
  typedef typename boost::graph_traits<GraphType>::vertex_iterator GraphVertexIterator;
  typedef typename boost::graph_traits<GraphType>::edge_iterator GraphEdgeIterator;

  // Visualize Edges
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns_prefix + "Edges";
  marker.id = 0;
// #define TRIANGLE
#ifdef TRIANGLE
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
#else
  marker.type = visualization_msgs::Marker::LINE_LIST;
#endif
  marker.action = visualization_msgs::Marker::ADD;
  
  GraphEdgeIterator it_edge, end_edges;
  for (boost::tie(it_edge,end_edges) = boost::edges(graph); it_edge!=end_edges; ++it_edge)
  {
#ifdef TRIANGLE
    geometry_msgs::Point point_start1;
    point_start1.x = graph[boost::source(*it_edge,graph)].pos[0]+0.05;
    point_start1.y = graph[boost::source(*it_edge,graph)].pos[1]-0.05;
    point_start1.z = 0;
    marker.points.push_back(point_start1);
    geometry_msgs::Point point_start2;
    point_start2.x = graph[boost::source(*it_edge,graph)].pos[0]-0.05;
    point_start2.y = graph[boost::source(*it_edge,graph)].pos[1]+0.05;
    point_start2.z = 0;
    marker.points.push_back(point_start2);

#else
    geometry_msgs::Point point_start;
    point_start.x = graph[boost::source(*it_edge,graph)].pos[0];
    point_start.y = graph[boost::source(*it_edge,graph)].pos[1];
    point_start.z = 0;
    marker.points.push_back(point_start);
#endif
    geometry_msgs::Point point_end;
    point_end.x = graph[boost::target(*it_edge,graph)].pos[0];
    point_end.y = graph[boost::target(*it_edge,graph)].pos[1];
    point_end.z = 0;
    marker.points.push_back(point_end);
    
    // add color
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 0;
    color.g = 0;
    color.b = 1;
    marker.colors.push_back(color);
    marker.colors.push_back(color);
#ifdef TRIANGLE
    marker.colors.push_back(color);
#endif
  }
  
#ifdef TRIANGLE
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
#else 
  marker.scale.x = 0.01;
#endif
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Now publish edge markers
  teb_marker_pub_.publish( marker );
  
  // Visualize vertices
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns_prefix + "Vertices";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  
  GraphVertexIterator it_vert, end_vert;
  for (boost::tie(it_vert,end_vert) = boost::vertices(graph); it_vert!=end_vert; ++it_vert)
  {
    geometry_msgs::Point point;
    point.x = graph[*it_vert].pos[0];
    point.y = graph[*it_vert].pos[1];
    point.z = 0;
    marker.points.push_back(point);
    // add color
    
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    if (it_vert==end_vert-1)
    {
      color.r = 1;
      color.g = 0;
      color.b = 0;		
    }
    else
    {
      color.r = 0;
      color.g = 1;
      color.b = 0;
    }
    marker.colors.push_back(color);
  }
  // set first color (start vertix) to blue
  if (!marker.colors.empty())
  {
    marker.colors.front().b = 1;
    marker.colors.front().g = 0;
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  // Now publish vertex markers
  teb_marker_pub_.publish( marker );
}
  
template <typename BidirIter>
void TebVisualization::publishPathContainer(BidirIter first, BidirIter last, const std::string& ns)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  typedef typename std::iterator_traits<BidirIter>::value_type PathType; // Get type of the path (point container)
  
  // Iterate through path container
  while(first != last)
  {	  
    // iterate single path points
    typename PathType::const_iterator it_point, end_point;
    for (it_point = first->begin(), end_point = boost::prior(first->end()); it_point != end_point; ++it_point) 
    {
      geometry_msgs::Point point_start;
      point_start.x = get_const_reference(*it_point).x();
      point_start.y = get_const_reference(*it_point).y();
      point_start.z = 0;
      marker.points.push_back(point_start);

      geometry_msgs::Point point_end;
      point_end.x = get_const_reference(*boost::next(it_point)).x();
      point_end.y = get_const_reference(*boost::next(it_point)).y();
      point_end.z = 0;
      marker.points.push_back(point_end);
    }
    ++first;
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_.publish( marker );
}
  
  
} // namespace teb_local_planner
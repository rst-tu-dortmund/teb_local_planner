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
  
//!< Inline function used for calculateHSignature() in combination with VertexPose pointers   
inline std::complex<long double> getCplxFromVertexPosePtr(const VertexPose* pose)
{
  return std::complex<long double>(pose->x(), pose->y());
};

//!< Inline function used for calculateHSignature() in combination with HCP graph vertex descriptors
inline std::complex<long double> getCplxFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return std::complex<long double>(graph[vert_descriptor].pos.x(), graph[vert_descriptor].pos.y());
};
  
//!< Inline function used for initializing the TEB in combination with HCP graph vertex descriptors
inline const Eigen::Vector2d& getVector2dFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return graph[vert_descriptor].pos;
};
  
//!< Inline function used for calculateHSignature() in combination with geometry_msgs::PoseStamped   
inline std::complex<long double> getCplxFromMsgPoseStamped(const geometry_msgs::PoseStamped& pose)
{
  return std::complex<long double>(pose.pose.position.x, pose.pose.position.y);
};


HomotopyClassPlanner::HomotopyClassPlanner() : obstacles_(NULL), via_points_(NULL),  cfg_(NULL), robot_model_(new PointRobotFootprint()),
                                               initial_plan_(NULL), initialized_(false)
{
}
  
HomotopyClassPlanner::HomotopyClassPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                           TebVisualizationPtr visual, const ViaPointContainer* via_points) : initial_plan_(NULL)
{
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

HomotopyClassPlanner::~HomotopyClassPlanner()
{
}

void HomotopyClassPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                      TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  robot_model_ = robot_model;
  initialized_ = true;
  
  setVisualization(visual);
}


void HomotopyClassPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}


 
bool HomotopyClassPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  
  // store initial plan for further initializations (must be valid for the lifetime of this object or clearPlanner() is called!)
  initial_plan_ = &initial_plan;
  // store the h signature of the initial plan to enable searching a matching teb later.
  initial_plan_h_sig_ = calculateHSignature(initial_plan.begin(), initial_plan.end(), getCplxFromMsgPoseStamped, obstacles_, cfg_->hcp.h_signature_prescaler);
    
  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);
  Eigen::Vector2d vel = start_vel ?  Eigen::Vector2d( start_vel->linear.x, start_vel->angular.z ) : Eigen::Vector2d::Zero();
  return plan(start, goal, vel, free_goal_vel);
}


bool HomotopyClassPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  PoseSE2 start_pose(start);
  PoseSE2 goal_pose(goal);
  Eigen::Vector2d vel = start_vel ?  Eigen::Vector2d( start_vel->linear.x, start_vel->angular.z ) : Eigen::Vector2d::Zero();
  return plan(start_pose, goal_pose, vel, free_goal_vel);
}

bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const Eigen::Vector2d& start_vel, bool free_goal_vel)
{	
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  
  // Update old TEBs with new start, goal and velocity
  updateAllTEBs(start, goal, start_vel);
    
  // Init new TEBs based on newly explored homotopy classes
  exploreHomotopyClassesAndInitTebs(start, goal, cfg_->obstacles.min_obstacle_dist, start_vel);
  // update via-points if activated
  updateReferenceTrajectoryViaPoints(cfg_->hcp.viapoints_all_candidates);
  // Optimize all trajectories in alternative homotopy classes
  optimizeAllTEBs(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  // Delete any detours
  deleteTebDetours(-0.1); 
  // Select which candidate (based on alternative homotopy classes) should be used
  selectBestTeb();

  initial_plan_ = NULL; // clear pointer to any previous initial plan (any previous plan is useless regarding the h-signature);
  return true;
} 
 
bool HomotopyClassPlanner::getVelocityCommand(double& v, double& omega) const
{
  TebOptimalPlannerConstPtr best_teb = bestTeb();
  if (!best_teb)
  {
    v = 0;
    omega = 0;
    return false;
  }
 
  return best_teb->getVelocityCommand(v, omega); 
}




void HomotopyClassPlanner::visualize()
{
  if (visualization_)
  {
    // Visualize graph
    if (cfg_->hcp.visualize_hc_graph)
      visualization_->publishGraph(graph_);
        
    // Visualize active tebs as marker
    visualization_->publishTebContainer(tebs_);
    
    // Visualize best teb and feedback message if desired
    TebOptimalPlannerConstPtr best_teb = bestTeb();
    if (best_teb)
    {
      visualization_->publishLocalPlanAndPoses(best_teb->teb());
      
      if (best_teb->teb().sizePoses() > 0) //TODO maybe store current pose (start) within plan method as class field.
        visualization_->publishRobotFootprintModel(best_teb->teb().Pose(0), *robot_model_);
    
      // feedback message
      if (cfg_->trajectory.publish_feedback)
      {
        int best_idx = bestTebIdx();
        if (best_idx>=0)
          visualization_->publishFeedbackMessage(tebs_, (unsigned int) best_idx, *obstacles_);
      }
    }
  }
  else ROS_DEBUG("Ignoring HomotopyClassPlanner::visualize() call, since no visualization class was instantiated before.");
}




void HomotopyClassPlanner::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  // Clear existing graph and paths
  clearGraph();
  
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  
  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance) 
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (tebs_.empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }
  
  Eigen::Vector2d normal(-diff[1],diff[0]); // normal-vector
  normal.normalize();
  normal = normal*dist_to_obst; // scale with obstacle_distance;
  
  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize();
  
  // store nearest obstacle keypoints -> only used if limit_obstacle_heading is enabled
  std::pair<HcGraphVertexType,HcGraphVertexType> nearest_obstacle; // both vertices are stored
  double min_dist = DBL_MAX;
  
  if (obstacles_!=NULL)
  {
    for (ObstContainer::const_iterator it_obst = obstacles_->begin(); it_obst != obstacles_->end(); ++it_obst)
    {
      // check if obstacle is placed in front of start point
      Eigen::Vector2d start2obst = (*it_obst)->getCentroid() - start.position();
      double dist = start2obst.norm();
      if (start2obst.dot(diff)/dist<0.1)
        continue;
      
      // Add Keypoints	
      HcGraphVertexType u = boost::add_vertex(graph_);
      graph_[u].pos = (*it_obst)->getCentroid() + normal;
      HcGraphVertexType v = boost::add_vertex(graph_);
      graph_[v].pos = (*it_obst)->getCentroid() - normal;
      
      // store nearest obstacle
      if (obstacle_heading_threshold && dist<min_dist)
      {
        min_dist = dist;
        nearest_obstacle.first = u;
        nearest_obstacle.second = v;
      }
    }	
  }
  
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();
  
  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=end_i-1; ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j) 
        continue;
      // TODO: make use of knowing in which order obstacles are inserted and that for each obstacle 2 vertices are added,
      // therefore we must only check one of them.
      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize();
      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold)
        continue;

    
      // Check start angle to nearest obstacle 
      if (obstacle_heading_threshold && *it_i==start_vtx && min_dist!=DBL_MAX)
      {
        if (*it_j == nearest_obstacle.first || *it_j == nearest_obstacle.second)
        {
          Eigen::Vector2d keypoint_dist = graph_[*it_j].pos-start.position();
          keypoint_dist.normalize();
          Eigen::Vector2d start_orient_vec( cos(start.theta()), sin(start.theta()) ); // already normalized
          // check angle
          if (start_orient_vec.dot(keypoint_dist) <= obstacle_heading_threshold) 
          {
            ROS_DEBUG("createGraph() - deleted edge: limit_obstacle_heading");
            continue;
          }
        }
      }

      // Collision Check
      
      if (obstacles_!=NULL)
      {
        bool collision = false;
        for (ObstContainer::const_iterator it_obst = obstacles_->begin(); it_obst != obstacles_->end(); ++it_obst)
        {
          if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, 0.5*dist_to_obst) ) 
          {
            collision = true;
            break;
          }
        }
        if (collision) 
          continue;
      }
      
      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);			
    }
  }
  
   
  // Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity);
}


void HomotopyClassPlanner::createProbRoadmapGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, int no_samples,
                                                  double obstacle_heading_threshold, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  // Clear existing graph and paths
  clearGraph();
  
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  double start_goal_dist = diff.norm();
  
  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance) 
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (tebs_.empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }
  Eigen::Vector2d normal(-diff.coeffRef(1),diff.coeffRef(0)); // normal-vector
  normal.normalize();

  // Now sample vertices between start, goal and a specified width between both sides
  // Let's start with a square area between start and goal (maybe change it later to something like a circle or whatever)
  
  double area_width = cfg_->hcp.roadmap_graph_area_width;
    
  boost::random::uniform_real_distribution<double> distribution_x(0, start_goal_dist);  
  boost::random::uniform_real_distribution<double> distribution_y(0, area_width); 
  
  double phi = atan2(diff.coeffRef(1),diff.coeffRef(0)); // rotate area by this angle
  Eigen::Rotation2D<double> rot_phi(phi);
  
  Eigen::Vector2d area_origin = start.position() - 0.5*area_width*normal; // bottom left corner of the origin
  
  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize(); // normalize in place
  
  
  // Start sampling
  for (int i=0; i < no_samples; ++i)
  {
    Eigen::Vector2d sample;
//     bool coll_free;
//     do // sample as long as a collision free sample is found
//     {
      // Sample coordinates
      sample = area_origin + rot_phi*Eigen::Vector2d(distribution_x(rnd_generator_), distribution_y(rnd_generator_));
      
      // Test for collision
      // we do not care for collision checking here to improve efficiency, since we perform resampling repeatedly.
      // occupied vertices are ignored in the edge insertion state since they always violate the edge-obstacle collision check.
//       coll_free = true;
//       for (ObstContainer::const_iterator it_obst = obstacles_->begin(); it_obst != obstacles_->end(); ++it_obst)
//       {
//         if ( (*it_obst)->checkCollision(sample, dist_to_obst)) // TODO really keep dist_to_obst here?
//         {
//           coll_free = false;
//           break;
//         }
//       }
// 
//     } while (!coll_free && ros::ok());
    
    // Add new vertex
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = sample;
  }
  
  // Now add goal vertex
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();
  
  
  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i); ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j) // same vertex found
        continue;

      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize(); // normalize in place

      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold) 
          continue; // diff is already normalized
      

      // Collision Check	
      bool collision = false;
      for (ObstContainer::const_iterator it_obst = obstacles_->begin(); it_obst != obstacles_->end(); ++it_obst)
      {
        if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, dist_to_obst) )
        {
          collision = true;
          break;
        }
      }
      if (collision)
        continue;
      
      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);			
    }
  }
  
  /// Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity);
}


void HomotopyClassPlanner::DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal,
                                      double start_orientation, double goal_orientation, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths
  
  if ((int)tebs_.size() >= cfg_->hcp.max_number_classes)
    return; // We do not need to search for further possible alternative homotopy classes.
  
  HcGraphVertexType back = visited.back();

  /// Examine adjacent nodes
  HcGraphAdjecencyIterator it, end;
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() )
      continue; // already visited

    if ( *it == goal ) // goal reached
    {
      visited.push_back(*it);
      
      
      // check H-Signature
      std::complex<long double> H = calculateHSignature(visited.begin(), visited.end(), boost::bind(getCplxFromHcGraph, _1, boost::cref(graph_)), obstacles_, cfg_->hcp.h_signature_prescaler);
      
      // check if H-Signature is already known
      // and init new TEB if no duplicate was found
      if ( addHSignatureIfNew(H) )
      {
        addAndInitNewTeb(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)), start_orientation, goal_orientation, start_velocity);
      }
      
      visited.pop_back();
      break;
    }
}

/// Recursion for all adjacent vertices
for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
{
  if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() || *it == goal)
    continue; // already visited || goal reached
  
  
  visited.push_back(*it);
  
  // recursion step
  DepthFirst(g, visited, goal, start_orientation, goal_orientation, start_velocity);
  
  visited.pop_back();
}
}
 
 
bool HomotopyClassPlanner::hasHSignature(const std::complex<long double>& H) const
{
  // iterate existing h-signatures and check if there is an existing H-Signature similar the candidate
  for (std::vector< std::pair<std::complex<long double>, bool> >::const_iterator it = h_signatures_.begin(); it != h_signatures_.end(); ++it)
  {
    if (isHSignatureSimilar(it->first, H, cfg_->hcp.h_signature_threshold))
      return true; // Found! Homotopy class already exists, therefore nothing added  
  }
  return false;
}

bool HomotopyClassPlanner::addHSignatureIfNew(const std::complex<long double>& H, bool lock)
{	  
  if (!std::isfinite(H.real()) || !std::isfinite(H.imag()))
  {
    ROS_WARN("HomotopyClassPlanner: Ignoring nan/inf H-signature");
    return false;
  }
  
  if (hasHSignature(H))
    return false;

  // Homotopy class not found -> Add to class-list, return that the h-signature is new
  h_signatures_.push_back(std::make_pair(H,lock));	 
  return true;
}
 
 

  
//! Small helper function to check whether two h-signatures are assumed to be equal. 
inline bool compareH( std::pair<TebOptPlannerContainer::iterator, std::complex<long double> > i, std::complex<long double> j ) {return std::abs(i.second.real()-j.real())<0.1 && std::abs(i.second.imag()-j.imag())<0.1;};
 

void HomotopyClassPlanner::renewAndAnalyzeOldTebs(bool delete_detours)
{
  // clear old h-signatures (since they could be changed due to new obstacle positions.
  h_signatures_.clear();

  // Collect h-signatures for all existing TEBs and store them together with the corresponding iterator / pointer:
//   typedef std::list< std::pair<TebOptPlannerContainer::iterator, std::complex<long double> > > TebCandidateType;
//   TebCandidateType teb_candidates;

  // get new homotopy classes and delete multiple TEBs per homotopy class
  TebOptPlannerContainer::iterator it_teb = tebs_.begin();
  while(it_teb != tebs_.end())
  {
    // delete Detours if there is at least one other TEB candidate left in the container
    if (delete_detours && tebs_.size()>1 && it_teb->get()->teb().detectDetoursBackwards(-0.1)) 
    {
      it_teb = tebs_.erase(it_teb); // delete candidate and set iterator to the next valid candidate
      continue;
    }

    // calculate H Signature for the current candidate
    std::complex<long double> H = calculateHSignature(it_teb->get()->teb().poses().begin(), it_teb->get()->teb().poses().end(), getCplxFromVertexPosePtr ,obstacles_, cfg_->hcp.h_signature_prescaler);
    
//     teb_candidates.push_back(std::make_pair(it_teb,H));
    
    // WORKAROUND until the commented code below works
    // Here we do not compare cost values. Just first come first serve...
    bool new_flag = addHSignatureIfNew(H);
    if (!new_flag)
    {
      it_teb = tebs_.erase(it_teb);
      continue;
    }
    
    ++it_teb;
  }

  // Find multiple candidates and delete the one with higher cost 
  // TODO: this code needs to be adpated. Erasing tebs from the teb container_ could make iteratores stored in the candidate list invalid!
//   TebCandidateType::reverse_iterator cand_i = teb_candidates.rbegin();
//   int test_idx = 0;
//   while (cand_i != teb_candidates.rend())
//   {
//          
//     TebCandidateType::reverse_iterator cand_j = std::find_if(boost::next(cand_i),teb_candidates.rend(), boost::bind(compareH,_1,cand_i->second));
//     if (cand_j != teb_candidates.rend() && cand_j != cand_i)
//     {
//         TebOptimalPlannerPtr pt1 = *(cand_j->first);
//         TebOptimalPlannerPtr pt2 = *(cand_i->first);
//         assert(pt1);
//         assert(pt2);
//       if ( cand_j->first->get()->getCurrentCost().sum() > cand_i->first->get()->getCurrentCost().sum() )
//       {
// 	// found one that has higher cost, therefore erase cand_j
// 	tebs_.erase(cand_j->first);
// 	teb_candidates.erase(cand_j);         
//       }
//       else   // otherwise erase cand_i
//       {
// 	tebs_.erase(cand_i->first);
// 	cand_i = teb_candidates.erase(cand_i);
//       }
//     }
//     else 
//     {
//         ROS_WARN_STREAM("increase cand_i");
//         ++cand_i;	
//     }
//   }
  
  // now add the h-signatures to the internal lookup-table (but only if there is no existing duplicate)
//   for (TebCandidateType::iterator cand=teb_candidates.begin(); cand!=teb_candidates.end(); ++cand)
//   {
//     bool new_flag = addNewHSignatureIfNew(cand->second, cfg_->hcp.h_signature_threshold);
//     if (!new_flag)
//     {
// //       ROS_ERROR_STREAM("getAndFilterHomotopyClassesTEB() - This schould not be happen.");
//       tebs_.erase(cand->first);
//     }
//   }
	
}
 
void HomotopyClassPlanner::updateReferenceTrajectoryViaPoints(bool all_trajectories)
{
  if ( (!all_trajectories && !initial_plan_) || !via_points_ || via_points_->empty() || cfg_->optim.weight_viapoint <= 0)
    return;
  
  if(h_signatures_.size() < tebs_.size())
  {
    ROS_ERROR("HomotopyClassPlanner::updateReferenceTrajectoryWithViaPoints(): Number of h-signatures does not match number of trajectories.");
    return;
  }
  
  if (all_trajectories)
  {
    // enable via-points for all tebs
    for (std::size_t i=0; i < h_signatures_.size(); ++i)
    {
        tebs_[i]->setViaPoints(via_points_);
    }
  }
  else
  {
    // enable via-points for teb in the same hommotopy class as the initial_plan and deactivate it for all other ones
    for (std::size_t i=0; i < h_signatures_.size(); ++i)
    {
      if (isHSignatureSimilar(h_signatures_[i].first, initial_plan_h_sig_, cfg_->hcp.h_signature_threshold))
        tebs_[i]->setViaPoints(via_points_);
      else
        tebs_[i]->setViaPoints(NULL);
    }
  }
}
 
 
void HomotopyClassPlanner::exploreHomotopyClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, boost::optional<const Eigen::Vector2d&> start_vel)
{
  // first process old trajectories
  renewAndAnalyzeOldTebs(false);

  // inject initial plan if available and not yet captured
  if (initial_plan_ && addHSignatureIfNew(initial_plan_h_sig_, true)) // also prevent candidate from deletion
    addAndInitNewTeb(*initial_plan_, start_vel);
    
  // now explore new homotopy classes and initialize tebs if new ones are found.
  if (cfg_->hcp.simple_exploration)
    createGraph(start,goal,dist_to_obst,cfg_->hcp.obstacle_heading_threshold, start_vel);
  else
    createProbRoadmapGraph(start,goal,dist_to_obst,cfg_->hcp.roadmap_graph_no_samples,cfg_->hcp.obstacle_heading_threshold, start_vel);
} 


void HomotopyClassPlanner::addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  tebs_.push_back( TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_) ) );
  tebs_.back()->teb().initTEBtoGoal(start, goal, 0, cfg_->trajectory.dt_ref, cfg_->trajectory.min_samples);
  
  if (start_velocity)
    tebs_.back()->setVelocityStart(*start_velocity);
}

void HomotopyClassPlanner::addAndInitNewTeb(const std::vector<geometry_msgs::PoseStamped>& initial_plan, boost::optional<const Eigen::Vector2d&> start_velocity)
{
  tebs_.push_back( TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_) ) );
  tebs_.back()->teb().initTEBtoGoal(*initial_plan_, cfg_->trajectory.dt_ref, true, cfg_->trajectory.min_samples); 
  
  if (start_velocity)
    tebs_.back()->setVelocityStart(*start_velocity);
}

void HomotopyClassPlanner::updateAllTEBs(boost::optional<const PoseSE2&> start, boost::optional<const PoseSE2&> goal,  boost::optional<const Eigen::Vector2d&> start_velocity)
{
  // If new goal is too far away, clear all existing trajectories to let them reinitialize later.
  // Since all Tebs are sharing the same fixed goal pose, just take the first candidate:
  if (!tebs_.empty() && (goal->position() - tebs_.front()->teb().BackPose().position()).norm() >= cfg_->trajectory.force_reinit_new_goal_dist)
  {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      tebs_.clear();
      h_signatures_.clear();
  }  
  
  // hot-start from previous solutions
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->teb().updateAndPruneTEB(start, goal);
    if (start_velocity)
      it_teb->get()->setVelocityStart(*start_velocity);
  }
}

 
void HomotopyClassPlanner::optimizeAllTEBs(unsigned int iter_innerloop, unsigned int iter_outerloop)
{
  // optimize TEBs in parallel since they are independend of each other
  if (cfg_->hcp.enable_multithreading)
  {
    boost::thread_group teb_threads;
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      teb_threads.create_thread( boost::bind(&TebOptimalPlanner::optimizeTEB, it_teb->get(), iter_innerloop, iter_outerloop,
                                             true, cfg_->hcp.selection_obst_cost_scale, cfg_->hcp.selection_viapoint_cost_scale,
                                             cfg_->hcp.selection_alternative_time_cost) );
    }
    teb_threads.join_all();
  }
  else
  {
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      it_teb->get()->optimizeTEB(iter_innerloop,iter_outerloop, true, cfg_->hcp.selection_obst_cost_scale, 
                                 cfg_->hcp.selection_viapoint_cost_scale, cfg_->hcp.selection_alternative_time_cost); // compute cost as well inside optimizeTEB (last argument = true)
    }
  }
} 
 
void HomotopyClassPlanner::deleteTebDetours(double threshold)
{
  TebOptPlannerContainer::iterator it_teb = tebs_.begin();
  bool modified;
  int h_idx = 0; // keep track about the current h-signature idx
  while(it_teb != tebs_.end())
  { 
    modified = false;
    
    if (!h_signatures_[h_idx].second) // check if h-signature is locked
    {
      // delete Detours if other TEBs will remain!
      if (tebs_.size()>1 && it_teb->get()->teb().detectDetoursBackwards(threshold))
      {
	it_teb = tebs_.erase(it_teb); // 0.05
	modified = true;
      }
    }
    
    // Also delete tebs that cannot be optimized (last optim call failed)
    // here, we ignore the lock-state, since we cannot keep trajectories that are not optimizable
    if (!it_teb->get()->isOptimized())
    {
	it_teb = tebs_.erase(it_teb);
	modified = true;      
    }  
    
    if (!modified)
       ++it_teb;
    
    ++h_idx;
  }
} 
 
 
TebOptimalPlannerPtr HomotopyClassPlanner::selectBestTeb()
{
  double min_cost = std::numeric_limits<double>::max(); // maximum cost
  
  // check if last best_teb is still a valid candidate
  if (std::find(tebs_.begin(), tebs_.end(), best_teb_) != tebs_.end())
  {
    // get cost of this candidate
    min_cost = best_teb_->getCurrentCost() * cfg_->hcp.selection_cost_hysteresis; // small hysteresis
  }
  else // the last candidate is not valid anymore
    best_teb_.reset(); // reset pointer

  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    if (*it_teb == best_teb_)
      continue; // skip already known cost value of the last best_teb
    
    double teb_cost = it_teb->get()->getCurrentCost();

    if (teb_cost < min_cost)
    {
      // check if this candidate is currently not selected
      best_teb_ = *it_teb;
      min_cost = teb_cost;
    }
  }	
  return best_teb_;
} 

int HomotopyClassPlanner::bestTebIdx() const
{
  if (tebs_.size() == 1)
    return 0;
    
  if (!best_teb_)
    return -1;
  
  int idx = 0;
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb, ++idx)
  {
    if (*it_teb == best_teb_)
      return idx;
  }
  return -1;  
}

bool HomotopyClassPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                                double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  TebOptimalPlannerPtr best = bestTeb();
  if (!best)
    return false;
  
  return best->isTrajectoryFeasible(costmap_model,footprint_spec, inscribed_radius, circumscribed_radius, look_ahead_idx);
}

bool HomotopyClassPlanner::isHorizonReductionAppropriate(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const
{
  TebOptimalPlannerPtr best = bestTeb();
  if (!best)
    return false;
  
  return best->isHorizonReductionAppropriate(initial_plan);
}

void HomotopyClassPlanner::computeCurrentCost(std::vector<double>& cost, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->computeCurrentCost(cost, obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
  } 
}
 
 
} // end namespace

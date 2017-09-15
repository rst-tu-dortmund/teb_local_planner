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

HomotopyClassPlanner::HomotopyClassPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), robot_model_(new PointRobotFootprint()), initial_plan_(NULL), initialized_(false)
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

  if (cfg_->hcp.simple_exploration)
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new lrKeyPointGraph(*cfg_, this));
  else
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new ProbRoadmapGraph(*cfg_, this));

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

  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);

  return plan(start, goal, start_vel, free_goal_vel);
}


bool HomotopyClassPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  PoseSE2 start_pose(start);
  PoseSE2 goal_pose(goal);
  return plan(start_pose, goal_pose, start_vel, free_goal_vel);
}

bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  // Update old TEBs with new start, goal and velocity
  updateAllTEBs(&start, &goal, start_vel);

  // Init new TEBs based on newly explored homotopy classes
  exploreEquivalenceClassesAndInitTebs(start, goal, cfg_->obstacles.min_obstacle_dist, start_vel);
  // update via-points if activated
  updateReferenceTrajectoryViaPoints(cfg_->hcp.viapoints_all_candidates);
  // Optimize all trajectories in alternative homotopy classes
  optimizeAllTEBs(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  // Delete any detours
  deleteTebDetours(-0.1);
  // Select which candidate (based on alternative homotopy classes) should be used
  selectBestTeb();

  initial_plan_ = nullptr; // clear pointer to any previous initial plan (any previous plan is useless regarding the h-signature);
  return true;
}

bool HomotopyClassPlanner::getVelocityCommand(double& vx, double& vy, double& omega) const
{
  TebOptimalPlannerConstPtr best_teb = bestTeb();
  if (!best_teb)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  return best_teb->getVelocityCommand(vx, vy, omega);
}




void HomotopyClassPlanner::visualize()
{
  if (visualization_)
  {
    // Visualize graph
    if (cfg_->hcp.visualize_hc_graph && graph_search_)
      visualization_->publishGraph(graph_search_->graph_);

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



bool HomotopyClassPlanner::hasEquivalenceClass(const EquivalenceClassPtr& eq_class) const
{
  // iterate existing h-signatures and check if there is an existing H-Signature similar the candidate
  for (const std::pair<EquivalenceClassPtr, bool>& eqrel : equivalence_classes_)
  {
     if (eq_class->isEqual(*eqrel.first))
        return true; // Found! Homotopy class already exists, therefore nothing added
  }
  return false;
}

bool HomotopyClassPlanner::addEquivalenceClassIfNew(const EquivalenceClassPtr& eq_class, bool lock)
{
  if (!eq_class)
    return false;

  if (!eq_class->isValid())
  {
    ROS_WARN("HomotopyClassPlanner: Ignoring invalid H-signature");
    return false;
  }

  if (hasEquivalenceClass(eq_class))
    return false;

  // Homotopy class not found -> Add to class-list, return that the h-signature is new
  equivalence_classes_.push_back(std::make_pair(eq_class,lock));
  return true;
}


void HomotopyClassPlanner::renewAndAnalyzeOldTebs(bool delete_detours)
{
  // clear old h-signatures (since they could be changed due to new obstacle positions.
  equivalence_classes_.clear();

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

    // calculate equivalence class for the current candidate
    EquivalenceClassPtr equivalence_class = calculateEquivalenceClass(it_teb->get()->teb().poses().begin(), it_teb->get()->teb().poses().end(), getCplxFromVertexPosePtr , obstacles_,
                                                                      it_teb->get()->teb().timediffs().begin(), it_teb->get()->teb().timediffs().end());

//     teb_candidates.push_back(std::make_pair(it_teb,H));

    // WORKAROUND until the commented code below works
    // Here we do not compare cost values. Just first come first serve...
    bool new_flag = addEquivalenceClassIfNew(equivalence_class);
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

  if(equivalence_classes_.size() < tebs_.size())
  {
    ROS_ERROR("HomotopyClassPlanner::updateReferenceTrajectoryWithViaPoints(): Number of h-signatures does not match number of trajectories.");
    return;
  }

  if (all_trajectories)
  {
    // enable via-points for all tebs
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
        tebs_[i]->setViaPoints(via_points_);
    }
  }
  else
  {
    // enable via-points for teb in the same hommotopy class as the initial_plan and deactivate it for all other ones
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
      if(initial_plan_eq_class_->isEqual(*equivalence_classes_[i].first))
        tebs_[i]->setViaPoints(via_points_);
      else
        tebs_[i]->setViaPoints(NULL);
    }
  }
}


void HomotopyClassPlanner::exploreEquivalenceClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, const geometry_msgs::Twist* start_vel)
{
  // first process old trajectories
  renewAndAnalyzeOldTebs(false);

  // inject initial plan if available and not yet captured
  if (initial_plan_)
  {
    initial_plan_teb_ = addAndInitNewTeb(*initial_plan_, start_vel);
  }
  else
  {
    initial_plan_teb_.reset();
    initial_plan_teb_ = getInitialPlanTEB(); // this method searches for initial_plan_eq_class_ in the teb container (-> if !initial_plan_teb_)
  }

  // now explore new homotopy classes and initialize tebs if new ones are found. The appropriate createGraph method is chosen via polymorphism.
  graph_search_->createGraph(start,goal,dist_to_obst,cfg_->hcp.obstacle_heading_threshold, start_vel);
}


TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_velocity)
{
  TebOptimalPlannerPtr candidate =  TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_));

  candidate->teb().initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                    candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

  if(addEquivalenceClassIfNew(H))
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}


TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_velocity)
{
  TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_));

  candidate->teb().initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, true, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  // store the h signature of the initial plan to enable searching a matching teb later.
  initial_plan_eq_class_ = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                     candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

  if(addEquivalenceClassIfNew(initial_plan_eq_class_, true)) // also prevent candidate from deletion
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}

void HomotopyClassPlanner::updateAllTEBs(const PoseSE2* start, const PoseSE2* goal, const geometry_msgs::Twist* start_velocity)
{
  // If new goal is too far away, clear all existing trajectories to let them reinitialize later.
  // Since all Tebs are sharing the same fixed goal pose, just take the first candidate:
  if (!tebs_.empty() && (goal->position() - tebs_.front()->teb().BackPose().position()).norm() >= cfg_->trajectory.force_reinit_new_goal_dist)
  {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      tebs_.clear();
      equivalence_classes_.clear();
  }

  // hot-start from previous solutions
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->teb().updateAndPruneTEB(*start, *goal);
    if (start_velocity)
      it_teb->get()->setVelocityStart(*start_velocity);
  }
}


void HomotopyClassPlanner::optimizeAllTEBs(int iter_innerloop, int iter_outerloop)
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
  EquivalenceClassContainer::iterator it_eqclasses = equivalence_classes_.begin();

  if (tebs_.size() != equivalence_classes_.size())
  {
    ROS_ERROR("HomotopyClassPlanner::deleteTebDetours(): number of equivalence classes (%lu) and trajectories (%lu) does not match.", equivalence_classes_.size(), tebs_.size());
    return;
  }

  bool modified;

  while(it_teb != tebs_.end())
  {
    modified = false;

    if (!it_eqclasses->second) // check if equivalence class is locked
    {
      // delete Detours if other TEBs will remain!
      if (tebs_.size()>1 && (it_teb->get()->teb().detectDetoursBackwards(threshold) || !it_eqclasses->first->isReasonable()))
      {
        it_teb = tebs_.erase(it_teb);
        it_eqclasses = equivalence_classes_.erase(it_eqclasses);
        modified = true;
      }
    }

    // Also delete tebs that cannot be optimized (last optim call failed)
    // here, we ignore the lock-state, since we cannot keep trajectories that are not optimizable
    if (!it_teb->get()->isOptimized())
    {
      it_teb = tebs_.erase(it_teb);
      it_eqclasses = equivalence_classes_.erase(it_eqclasses);
      modified = true;
      ROS_DEBUG("HomotopyClassPlanner::deleteTebDetours(): removing candidate that was not optimized successfully");
    }

    if (!modified)
    {
      ++it_teb;
      ++it_eqclasses;
    }
  }
}

TebOptimalPlannerPtr HomotopyClassPlanner::getInitialPlanTEB()
{
    // first check stored teb object
    if (initial_plan_teb_)
    {
        // check if the teb is still part of the teb container
        if ( std::find(tebs_.begin(), tebs_.end(), initial_plan_teb_ ) != tebs_.end() )
            return initial_plan_teb_;
        else
        {
            initial_plan_teb_.reset(); // reset pointer for next call
            ROS_DEBUG("initial teb not found, trying to find a match according to the cached equivalence class");
        }
    }

    // reset the locked state for equivalence classes // TODO: this might be adapted if not only the plan containing the initial plan is locked!
    for (int i=0; i<equivalence_classes_.size(); ++i)
    {
        equivalence_classes_[i].second = false;
    }

    // otherwise check if the stored reference equivalence class exist in the list of known classes
    if (initial_plan_eq_class_ && initial_plan_eq_class_->isValid())
    {
         if (equivalence_classes_.size() == tebs_.size())
         {
            for (int i=0; i<equivalence_classes_.size(); ++i)
            {
                if (equivalence_classes_[i].first->isEqual(*initial_plan_eq_class_))
                {
                    equivalence_classes_[i].second = true;
                    return tebs_[i];
                }
            }
         }
         else
             ROS_ERROR("HomotopyClassPlanner::getInitialPlanTEB(): number of equivalence classes (%lu) and number of trajectories (%lu) does not match.", equivalence_classes_.size(), tebs_.size());
    }
    else
        ROS_DEBUG("HomotopyClassPlanner::getInitialPlanTEB(): initial TEB not found in the set of available trajectories.");

    return TebOptimalPlannerPtr();
}

TebOptimalPlannerPtr HomotopyClassPlanner::selectBestTeb()
{
    double min_cost = std::numeric_limits<double>::max(); // maximum cost
    double min_cost_last_best = std::numeric_limits<double>::max();
    double min_cost_initial_plan_teb = std::numeric_limits<double>::max();
    TebOptimalPlannerPtr last_best_teb;
    TebOptimalPlannerPtr initial_plan_teb = getInitialPlanTEB();

    // check if last best_teb is still a valid candidate
    if (std::find(tebs_.begin(), tebs_.end(), best_teb_) != tebs_.end())
    {
        // get cost of this candidate
        min_cost_last_best = best_teb_->getCurrentCost() * cfg_->hcp.selection_cost_hysteresis; // small hysteresis
        last_best_teb = best_teb_;
    }

    // check if last best_teb is still a valid candidate
    if (initial_plan_teb) // the validity was already checked in getInitialPlanTEB()
    {
        // get cost of this candidate
        min_cost_initial_plan_teb = initial_plan_teb->getCurrentCost() * cfg_->hcp.selection_prefer_initial_plan; // small hysteresis
    }


    best_teb_.reset(); // reset pointer

    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
        // check if the related TEB leaves the local costmap region
//      if (tebs_.size()>1 && !(*it_teb)->teb().isTrajectoryInsideRegion(20, -1, 1))
//      {
//          ROS_INFO("HomotopyClassPlanner::selectBestTeb(): skipping trajectories that are not inside the local costmap");
//          continue;
//      }

        double teb_cost;

        if (*it_teb == last_best_teb)
            teb_cost = min_cost_last_best; // skip already known cost value of the last best_teb
        else if (*it_teb == initial_plan_teb)
            teb_cost = min_cost_initial_plan_teb;
        else
            teb_cost = it_teb->get()->getCurrentCost();

        if (teb_cost < min_cost)
        {
        // check if this candidate is currently not selected
        best_teb_ = *it_teb;
        min_cost = teb_cost;
        }
     }


  // in case we haven't found any teb due to some previous checks, investigate list again
//   if (!best_teb_ && !tebs_.empty())
//   {
//       ROS_DEBUG("all " << tebs_.size() << " tebs rejected previously");
//       if (tebs_.size()==1)
//         best_teb_ = tebs_.front();
//       else // if multiple TEBs are available:
//       {
//           // try to use the one that relates to the initial plan
//           TebOptimalPlannerPtr initial_plan_teb = getInitialPlanTEB();
//           if (initial_plan_teb)
//               best_teb_ = initial_plan_teb;
//           else
//           {
//              // now compute the cost for the rest (we haven't computed it before)
//              for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
//              {
//                 double teb_cost = it_teb->get()->getCurrentCost();
//                 if (teb_cost < min_cost)
//                 {
//                     // check if this candidate is currently not selected
//                     best_teb_ = *it_teb;
//                     min_cost = teb_cost;
//                 }
//              }
//           }
//       }
//   }

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

void HomotopyClassPlanner::setPreferredTurningDir(RotType dir)
{
  // set preferred turning dir for all TEBs
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    (*it_teb)->setPreferredTurningDir(dir);
  }
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

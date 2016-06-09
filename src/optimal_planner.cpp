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

#include <teb_local_planner/optimal_planner.h>


namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  setVisualization(visual);
  
  vel_start_.first = true;
  vel_start_.second.setZero();

  vel_goal_.first = true;
  vel_goal_.second.setZero();
  initialized_ = true;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(unsigned int iterations_innerloop, unsigned int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false) 
    return false;
  bool success = false;
  optimized_ = false;
  for(unsigned int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples);

    success = buildGraph();
    if (!success) 
    {
        clearGraph();
        return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
      
    clearGraph();
  }

  return true;
}

void TebOptimalPlanner::setVelocityStart(const Eigen::Ref<const Eigen::Vector2d>& vel_start)
{
  vel_start_.first = true;
  vel_start_.second = vel_start;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.coeffRef(0) = vel_start.linear.x;
  vel_start_.second.coeffRef(1) = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const Eigen::Ref<const Eigen::Vector2d>& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTEBtoGoal(initial_plan, cfg_->trajectory.dt_ref, true, cfg_->trajectory.min_samples);
  } 
  else // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses()>0 && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTEBtoGoal(initial_plan, cfg_->trajectory.dt_ref, true, cfg_->trajectory.min_samples);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  Eigen::Vector2d vel = start_vel ? Eigen::Vector2d(start_vel->linear.x, start_vel->angular.z) : Eigen::Vector2d::Zero();
  return plan(start_, goal_, vel);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const Eigen::Vector2d& start_vel, bool free_goal_vel)
{	
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTEBtoGoal(start, goal, 0, 1, cfg_->trajectory.min_samples); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses()>0 && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTEBtoGoal(start, goal, 0, 1, cfg_->trajectory.min_samples);
    }
  }
  setVelocityStart(start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::buildGraph()
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  
  // add TEB vertices
  AddTEBVertices();
  
  // add Edges (local cost functions)
  if (cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist)
    AddEdgesInflatedObstacles();
  else
    AddEdgesObstacles();
  
  //AddEdgesDynamicObstacles();
  
  AddEdgesViaPoints();
  
  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	
  
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || (int)teb_.sizePoses()<cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);
  
  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
  optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
  optimizer_->clear();	
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  for (unsigned int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
  } 
}



void TebOptimalPlanner::AddEdgesObstacles()
{
  if (cfg_->optim.weight_obstacle==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle);
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if ((*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    unsigned int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= (int)teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
    dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
    dist_bandpt_obst->setInformation(information);
    dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
    optimizer_->addEdge(dist_bandpt_obst);

    for (unsigned int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
        EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
        dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
        dist_bandpt_obst_n_r->setInformation(information);
        dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst_n_r);
      }
      if ( (int) index - (int) neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
        EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
        dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
        dist_bandpt_obst_n_l->setInformation(information);
        dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst_n_l);
      }
    } 
	  
  }
}

void TebOptimalPlanner::AddEdgesInflatedObstacles()
{
  if (cfg_->optim.weight_obstacle==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_obstacle;
  information(1,1) = cfg_->optim.weight_inflation;
  information(0,1) = information(1,0) = 0;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if ((*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
            continue; 
        
    EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
    dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
    dist_bandpt_obst->setInformation(information);
    dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
    optimizer_->addEdge(dist_bandpt_obst);

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
        EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
        dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
        dist_bandpt_obst_n_r->setInformation(information);
        dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst_n_r);
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
        EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
        dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
        dist_bandpt_obst_n_l->setInformation(information);
        dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst_n_l);
      }
    } 
          
  }
}


void TebOptimalPlanner::AddEdgesDynamicObstacles()
{
  if (cfg_->optim.weight_obstacle==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!
  
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_dynamic_obstacle);
  
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    for (std::size_t i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(i);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      //dynobst_edge->setVertex(1,teb.PointVertex(i+1));
      dynobst_edge->setVertex(1,teb_.TimeDiffVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setMeasurement(obst->get());
      dynobst_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(dynobst_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = (int)teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
      index = 1;
    
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
    return; // if weight equals zero skip adding edges!

  std::size_t NoBandpts(teb_.sizePoses());
  Eigen::Matrix<double,2,2> information;
  information.fill(0);
  information(0,0) = cfg_->optim.weight_max_vel_x;
  information(1,1) = cfg_->optim.weight_max_vel_theta;

  for (std::size_t i=0; i < NoBandpts - 1; ++i)
  {
    EdgeVelocity* velocity_edge = new EdgeVelocity;
    velocity_edge->setVertex(0,teb_.PoseVertex(i));
    velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
    velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
    velocity_edge->setInformation(information);
    velocity_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(velocity_edge);
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0 && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  std::size_t NoBandpts(teb_.sizePoses());
  Eigen::Matrix<double,2,2> information;
  information.fill(0);
  information(0,0) = cfg_->optim.weight_acc_lim_x;
  information(1,1) = cfg_->optim.weight_acc_lim_theta;
  
  // check if an initial velocity should be taken into accound
  if (vel_start_.first)
  {
    EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
    acceleration_edge->setVertex(0,teb_.PoseVertex(0));
    acceleration_edge->setVertex(1,teb_.PoseVertex(1));
    acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
    acceleration_edge->setInitialVelocity(vel_start_.second);
    acceleration_edge->setInformation(information);
    acceleration_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }

  // now add the usual acceleration edge for each tuple of three teb poses
  for (std::size_t i=0; i < NoBandpts - 2; ++i)
  {
    EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
    acceleration_edge->setVertex(0,teb_.PoseVertex(i));
    acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
    acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
    acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
    acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
    acceleration_edge->setInformation(information);
    acceleration_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }
  
  // check if a goal velocity should be taken into accound
  if (vel_goal_.first)
  {
    EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
    acceleration_edge->setVertex(0,teb_.PoseVertex(NoBandpts-2));
    acceleration_edge->setVertex(1,teb_.PoseVertex(NoBandpts-1));
    acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
    acceleration_edge->setGoalVelocity(vel_goal_.second);
    acceleration_edge->setInformation(information);
    acceleration_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(acceleration_edge);
  }  
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0) 
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (std::size_t i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (unsigned int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (unsigned int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    EdgeTimeOptimal* edge_time_optimal = dynamic_cast<EdgeTimeOptimal*>(*it);
    if (edge_time_optimal!=NULL && !alternative_time_cost)
    {
      cost_ += edge_time_optimal->getError().squaredNorm();
      continue;
    }

    EdgeKinematicsDiffDrive* edge_kinematics_dd = dynamic_cast<EdgeKinematicsDiffDrive*>(*it);
    if (edge_kinematics_dd!=NULL)
    {
      cost_ += edge_kinematics_dd->getError().squaredNorm();
      continue;
    }
    
    EdgeKinematicsCarlike* edge_kinematics_cl = dynamic_cast<EdgeKinematicsCarlike*>(*it);
    if (edge_kinematics_cl!=NULL)
    {
      cost_ += edge_kinematics_cl->getError().squaredNorm();
      continue;
    }
    
    EdgeVelocity* edge_velocity = dynamic_cast<EdgeVelocity*>(*it);
    if (edge_velocity!=NULL)
    {
      cost_ += edge_velocity->getError().squaredNorm();
      continue;
    }
    
    EdgeAcceleration* edge_acceleration = dynamic_cast<EdgeAcceleration*>(*it);
    if (edge_acceleration!=NULL)
    {
      cost_ += edge_acceleration->getError().squaredNorm();
      continue;
    }
    
    EdgeObstacle* edge_obstacle = dynamic_cast<EdgeObstacle*>(*it);
    if (edge_obstacle!=NULL)
    {
      cost_ += edge_obstacle->getError().squaredNorm() * obst_cost_scale;
      continue;
    }
    
    EdgeInflatedObstacle* edge_inflated_obstacle = dynamic_cast<EdgeInflatedObstacle*>(*it);
    if (edge_inflated_obstacle!=NULL)
    {
      cost_ += std::sqrt(std::pow(edge_inflated_obstacle->getError()[0],2) * obst_cost_scale 
               + std::pow(edge_inflated_obstacle->getError()[1],2));
      continue;
    }
    
    EdgeDynamicObstacle* edge_dyn_obstacle = dynamic_cast<EdgeDynamicObstacle*>(*it);
    if (edge_dyn_obstacle!=NULL)
    {
      cost_ += edge_dyn_obstacle->getError().squaredNorm() * obst_cost_scale;
      continue;
    }
    
    EdgeViaPoint* edge_viapoint = dynamic_cast<EdgeViaPoint*>(*it);
    if (edge_viapoint!=NULL)
    {
      cost_ += edge_viapoint->getError().squaredNorm() * viapoint_cost_scale;
      continue;
    }
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& v, double& omega) const
{
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
  // translational velocity
  double dir = deltaS.dot(conf1dir);
  v = (double) g2o::sign(dir) * deltaS.norm()/dt;
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& v, double& omega) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    v = 0;
    omega = 0;
    return false;
  }
  
  double dt = teb_.TimeDiff(0);
  if (dt<=0)
  {	
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    v = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(1), dt, v, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = (int) teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity
  velocity_profile.front().linear.y = velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.x();
  velocity_profile.front().angular.z = vel_start_.second.y();
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.y = velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.y = velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.x();
  velocity_profile.back().angular.z = vel_goal_.second.y();
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = (int) teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.y = start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.x();
  start.velocity.angular.z = vel_start_.second.y();
  start.time_from_start.fromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.y = point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1, vel2, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2, omega2);
    point.velocity.linear.x = 0.5*(vel1+vel2);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start.fromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.y = goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.x();
  goal.velocity.angular.z = vel_goal_.second.y();
  goal.time_from_start.fromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= (int) teb().sizePoses())
    look_ahead_idx = (int) teb().sizePoses()-1;
  
  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
      return false;
    
    // check if distance between two poses is higher than the robot radius and interpolate in that case
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      if ( (teb().Pose(i+1).position()-teb().Pose(i).position()).norm() > inscribed_radius)
      {
        // check one more time
        PoseSE2 center = PoseSE2::average(teb().Pose(i), teb().Pose(i+1));
        if ( costmap_model->footprintCost(center.x(), center.y(), center.theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
          return false;
      }
      
    }
  }
  return true;
}


bool TebOptimalPlanner::isHorizonReductionAppropriate(const std::vector<geometry_msgs::PoseStamped>& initial_plan) const
{
  if (teb_.sizePoses() < std::size_t( 1.5*double(cfg_->trajectory.min_samples) ) ) // trajectory is short already
    return false;
  
  // check if distance is at least 2m long // hardcoded for now
  double dist = 0;
  for (std::size_t i=1; i < teb_.sizePoses(); ++i)
  {
    dist += ( teb_.Pose(i).position() - teb_.Pose(i-1).position() ).norm();
    if (dist > 2)
      break;
  }
  if (dist <= 2)
    return false;
  
  // check if goal orientation is differing with more than 90° and the horizon is still long enough to exclude parking maneuvers.
  // use case: Sometimes the robot accomplish the following navigation task:
  // 1. wall following 2. 180° curve 3. following along the other side of the wall.
  // If the trajectory is too long, the trajectory might intersect with the obstace and the optimizer does 
  // push the trajectory to the correct side.
  if ( std::abs( g2o::normalize_theta( teb_.Pose(0).theta() - teb_.BackPose().theta() ) ) > M_PI/2)
  {
    ROS_DEBUG("TebOptimalPlanner::isHorizonReductionAppropriate(): Goal orientation - start orientation > 90° ");
    return true;
  }
  
  // check if goal heading deviates more than 90° w.r.t. start orienation
  if (teb_.Pose(0).orientationUnitVec().dot(teb_.BackPose().position() - teb_.Pose(0).position()) < 0)
  {
    ROS_DEBUG("TebOptimalPlanner::isHorizonReductionAppropriate(): Goal heading - start orientation > 90° ");
    return true;
  }
    
  // check ratio: distance along the inital plan and distance of the trajectory (maybe too much is cut off)
  int idx=0; // first get point close to the robot (should be fast if the global path is already pruned!)
  for (; idx < (int)initial_plan.size(); ++idx)
  {
    if ( std::sqrt(std::pow(initial_plan[idx].pose.position.x-teb_.Pose(0).x(), 2) + std::pow(initial_plan[idx].pose.position.y-teb_.Pose(0).y(), 2)) )
      break;
  } 
  // now calculate length
  double ref_path_length = 0;
  for (; idx < int(initial_plan.size())-1; ++idx)
  {
    ref_path_length += std::sqrt(std::pow(initial_plan[idx+1].pose.position.x-initial_plan[idx].pose.position.x, 2) 
                     + std::pow(initial_plan[idx+1].pose.position.y-initial_plan[idx].pose.position.y, 2) );
  } 
  
  // check distances along the teb trajectory (by the way, we also check if the distance between two poses is > obst_dist)
  double teb_length = 0;
  for (int i = 1; i < (int)teb_.sizePoses(); ++i )
  {
    double dist = (teb_.Pose(i).position() - teb_.Pose(i-1).position()).norm();
    if (dist > 0.95*cfg_->obstacles.min_obstacle_dist)
    {
      ROS_DEBUG("TebOptimalPlanner::isHorizonReductionAppropriate(): Distance between consecutive poses > 0.9*min_obstacle_dist");
      return true;
    }
    ref_path_length += dist;
  }
  if (ref_path_length>0 && teb_length/ref_path_length < 0.7) // now check ratio
  {
    ROS_DEBUG("TebOptimalPlanner::isHorizonReductionAppropriate(): Planned trajectory is at least 30° shorter than the initial plan");
    return true;
  }
  
  
  // otherwise we do not suggest shrinking the horizon:
  return false;
}

} // namespace teb_local_planner

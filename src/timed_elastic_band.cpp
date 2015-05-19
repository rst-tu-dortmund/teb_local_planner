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

#include <teb_local_planner/timed_elastic_band.h>


namespace teb_local_planner
{


TimedElasticBand::TimedElasticBand()
{		
}

TimedElasticBand::~TimedElasticBand()
{
  ROS_DEBUG("Destructor Timed_Elastic_Band...");
  clearTimedElasticBand();
}


void TimedElasticBand::addPose(const PoseSE2& pose, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

void TimedElasticBand::addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(position, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

 void TimedElasticBand::addPose(double x, double y, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

void TimedElasticBand::addTimeDiff(double dt, bool fixed)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
  timediff_vec_.push_back( timediff_vertex );
  return;
}


void TimedElasticBand::addPoseAndTimeDiff(double x, double y, double angle, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(x,y,angle,false);
    addTimeDiff(dt,false);
  }
  else 
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}


 
void TimedElasticBand::addPoseAndTimeDiff(const PoseSE2& pose, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(pose,false);
    addTimeDiff(dt,false);
  } else
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}

void TimedElasticBand::addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(position, theta,false);
    addTimeDiff(dt,false);
  } else 
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}


void TimedElasticBand::deletePose(unsigned int index)
{
  ROS_ASSERT(index<pose_vec_.size());
  delete pose_vec_.at(index);
  pose_vec_.erase(pose_vec_.begin()+index);
  return ;
}

void TimedElasticBand::deleteTimeDiff(unsigned int index)
{
  ROS_ASSERT(index<timediff_vec_.size());
  delete timediff_vec_.at(index);
  timediff_vec_.erase(timediff_vec_.begin()+index);
  return;
}

inline void TimedElasticBand::insertPose(unsigned int index, const PoseSE2& pose)
{
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
  return;
}

inline void TimedElasticBand::insertPose(unsigned int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
{
  VertexPose* pose_vertex = new VertexPose(position, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
  return;
}

inline void TimedElasticBand::insertPose(unsigned int index, double x, double y, double theta)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
  return;
}

inline void TimedElasticBand::insertTimeDiff(unsigned int index, double dt)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin()+index, timediff_vertex);
  return;
}


void TimedElasticBand::clearTimedElasticBand()
{
  for (PoseSequence::iterator pose_it = pose_vec_.begin(); pose_it != pose_vec_.end(); ++pose_it)
    delete *pose_it;
  pose_vec_.clear();
  
  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
    delete *dt_it;
  timediff_vec_.clear();
}


void TimedElasticBand::setPoseVertexFixed(unsigned int index, bool status)
{
  ROS_ASSERT(index<sizePoses());
  pose_vec_.at(index)->setFixed(status);   
  return;  
}

void TimedElasticBand::setTimeDiffVertexFixed(unsigned int index, bool status)
{
  ROS_ASSERT(index<sizeTimeDiffs());
  timediff_vec_.at(index)->setFixed(status);
  return;
}


void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis)
{
  /// iterate through all TEB states only once and add/remove states!
  for(unsigned int i=0; i < sizeTimeDiffs(); i++) // TimeDiff connects Point(i) with Point(i+1)
  {
    if(TimeDiff(i) > dt_ref + dt_hysteresis)
    {
      ROS_DEBUG("teb_local_planner: autoResize() inserting new bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());
      
      double newtime = 0.5*TimeDiff(i);

      TimeDiff(i) = newtime;
      insertPose(i+1, PoseSE2::average(Pose(i),Pose(i+1)) );
      insertTimeDiff(i+1,newtime);
    }
    
    else if(TimeDiff(i) < dt_ref - dt_hysteresis  && sizeTimeDiffs()>5)
    {
      ROS_DEBUG("teb_local_planner: autoResize() deleting bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());
      
      if(i < (sizeTimeDiffs()-1))
      {
	TimeDiff(i+1) = TimeDiff(i+1) + TimeDiff(i);
	deleteTimeDiff(i);
	deletePose(i+1);
      }
    }
  }
}


double TimedElasticBand::getSumOfAllTimeDiffs() const
{
  double time = 0;

  for(TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
  {
      time += (*dt_it)->dt();
  }
  return time;
}

bool TimedElasticBand::initTEBtoGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double timestep)
{
  if (!isInit())
  {   
    addPose(start); // add starting point
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization
	    
    if (diststep!=0)
    {
      Eigen::Vector2d point_to_goal = goal.position()-start.position();
      double dir_to_goal = std::atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
      double dx = diststep*std::cos(dir_to_goal);
      double dy = diststep*std::sin(dir_to_goal);
      
      double dist_to_goal = point_to_goal.norm();
      double no_steps_d = dist_to_goal/std::abs(diststep); // ignore negative values
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d);
      
      for (unsigned int i=1; i<=no_steps; i++) // start with 1! starting point had index 0
      {
	if (i==no_steps && no_steps_d==(float) no_steps) break; // if last conf (depending on stepsize) is equal to goal conf -> leave loop
	addPoseAndTimeDiff(start.x()+i*dx,start.y()+i*dy,dir_to_goal,timestep);
      }

    }
    addPoseAndTimeDiff(goal,timestep); // add goal point
    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization	
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d",(unsigned int) sizePoses(),(unsigned int) sizeTimeDiffs());
    return false;
  }
  return true;
}


bool TimedElasticBand::initTEBtoGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double dt)
{
  
  if (!isInit())
  {	
    addPose(plan.front().pose.position.x ,plan.front().pose.position.y, tf::getYaw(plan.front().pose.orientation)); // add starting point
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization
	    
    for (std::vector<geometry_msgs::PoseStamped>::const_iterator pose = plan.begin()+1; pose != plan.end(); ++pose)
    {
	addPoseAndTimeDiff(pose->pose.position.x, pose->pose.position.y, tf::getYaw(pose->pose.orientation),dt);
    }
    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d",(unsigned int) sizePoses(),(unsigned int) sizeTimeDiffs());
    return false;
  }
  
  return true;
}


unsigned int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point) const
{
  std::vector<double> dist_vec; // TODO: improve! efficiency
  dist_vec.reserve(sizePoses());
  // calc distances


  for (unsigned int i = 0; i < sizePoses(); i++)
  {
    Eigen::Vector2d diff = ref_point - Pose(i).position();
    dist_vec.push_back(diff.norm());
  }
  

  // find minimum
  unsigned int index_min = 0;

  double last_value = dist_vec.at(0);
  for (unsigned int i=1; i < dist_vec.size(); i++)
  {
    if (dist_vec.at(i) < last_value)
    {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  return index_min; // return index, because it's equal to the vertex, which represents this bandpoint
}


bool TimedElasticBand::detectDetoursBackwards(double threshold) const
{
  if (sizePoses()<2) return false;
  
  Eigen::Vector2d d_start_goal = BackPose().position() - Pose(0).position();
  d_start_goal.normalize(); // using scalar_product without normalizing vectors first result in different threshold-effects

  /// detect based on orientation
  for(unsigned int i=0; i < sizePoses(); ++i)
  {
    Eigen::Vector2d orient_vector(cos( Pose(i).theta() ), sin( Pose(i).theta() ) );
    if (orient_vector.dot(d_start_goal) < threshold)
    {	
      ROS_DEBUG("detectDetoursBackwards() - mark TEB for delete: start-orientation vs startgoal-vec");
      return true; // backward direction found
    }
  }
  
  /// check if upcoming configuration (next index) ist pushed behind the start (e.g. due to obstacles)
  // TODO: maybe we need a small hysteresis?
  for (unsigned int i=0;i<2;++i) // check only a few upcoming
  {
    if (i+1 >= sizePoses()) break;
    Eigen::Vector2d start2conf = Pose(i+1).position() - Pose(0).position();
    double dist = start2conf.norm();
    start2conf = start2conf/dist; // normalize -> we don't use start2conf.normalize() since we want to use dist later
    if (start2conf.dot(d_start_goal) < threshold && dist>0.01) // skip very small displacements
    {
      ROS_DEBUG("detectDetoursBackwards() - mark TEB for delete: curvature look-ahead relative to startconf");
      return true;
    }
  }	
  return false;
}




bool TimedElasticBand::updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, double max_goal_separation)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  if (new_start)
  {
    Pose(0) = *new_start;
    
    // delete bandpoints which are behind the new_start (in the past) to facilitate optimization process
    Eigen::Vector2d start2goal;
    if (new_goal) start2goal =  new_goal->position() - new_start->position();
    else start2goal = BackPose().position() - new_start->position();
    for (unsigned int i=1; i < sizePoses(); i++) // start already applied
    {
      Eigen::Vector2d start2conf = Pose(i).position() - new_start->position();
      if(start2goal.dot(start2conf)<=0)
      {
	      ROS_DEBUG("updateAndPruneTEB() - Bandpoint in the past detected. Deleting %u ...",i);
	      // setTimeDiff(i,TimeDiff(i-1)+TimeDiff(i));
	      deleteTimeDiff(i-1);
	      deletePose(i);
      }
      else break;
    }
  }
  
  if (new_goal)
  {
    unsigned int teb_size = sizePoses();
    if((BackPose().position()-new_goal->position()).squaredNorm() > max_goal_separation*max_goal_separation) return false;
    
    BackPose() = *new_goal;
  }

  return true;
};





} // namespace teb_local_planner
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

#include "teb_local_planner/timed_elastic_band.h"

namespace teb_local_planner
{

namespace
{
  /**
   * estimate the time to move from start to end.
   * Assumes constant velocity for the motion.
   */
  double estimateDeltaT(const PoseSE2& start, const PoseSE2& end,
                        double max_vel_x, double max_vel_theta)
  {
    double dt_constant_motion = 0.1;
    if (max_vel_x > 0) {
      double trans_dist = (end.position() - start.position()).norm();
      dt_constant_motion = trans_dist / max_vel_x;
    }
    if (max_vel_theta > 0) {
      double rot_dist = std::abs(g2o::normalize_theta(end.theta() - start.theta()));
      dt_constant_motion = std::max(dt_constant_motion, rot_dist / max_vel_theta);
    }
    return dt_constant_motion;
  }
} // namespace


TimedElasticBand::TimedElasticBand()
{
}

TimedElasticBand::~TimedElasticBand()
{
  RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"), "Destructor Timed_Elastic_Band...");
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
  assert(dt > 0.0 && "Adding a timediff requires a positive dt");
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
  else {
    RCLCPP_ERROR(rclcpp::get_logger("teb_local_planner"),
                 "Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  }
  return;
}



void TimedElasticBand::addPoseAndTimeDiff(const PoseSE2& pose, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(pose,false);
    addTimeDiff(dt,false);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("teb_local_planner"), "Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  }
  return;
}

void TimedElasticBand::addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(position, theta,false);
    addTimeDiff(dt,false);
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"),
                 "Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  }
  return;
}


void TimedElasticBand::deletePose(int index)
{
  assert(index<pose_vec_.size());
  delete pose_vec_.at(index);
  pose_vec_.erase(pose_vec_.begin()+index);
}

void TimedElasticBand::deletePoses(int index, int number)
{
  assert(index+number<=(int)pose_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete pose_vec_.at(i);
  pose_vec_.erase(pose_vec_.begin()+index, pose_vec_.begin()+index+number);
}

void TimedElasticBand::deleteTimeDiff(int index)
{
  assert(index<(int)timediff_vec_.size());
  delete timediff_vec_.at(index);
  timediff_vec_.erase(timediff_vec_.begin()+index);
}

void TimedElasticBand::deleteTimeDiffs(int index, int number)
{
  assert(index+number<=timediff_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete timediff_vec_.at(i);
  timediff_vec_.erase(timediff_vec_.begin()+index, timediff_vec_.begin()+index+number);
}

void TimedElasticBand::insertPose(int index, const PoseSE2& pose)
{
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
{
  VertexPose* pose_vertex = new VertexPose(position, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, double x, double y, double theta)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertTimeDiff(int index, double dt)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin()+index, timediff_vertex);
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


void TimedElasticBand::setPoseVertexFixed(int index, bool status)
{
  assert(index<sizePoses());
  pose_vec_.at(index)->setFixed(status);
}

void TimedElasticBand::setTimeDiffVertexFixed(int index, bool status)
{
  assert(index<sizeTimeDiffs());
  timediff_vec_.at(index)->setFixed(status);
}


void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
{
  assert(sizeTimeDiffs() == 0 || sizeTimeDiffs() + 1 == sizePoses());
  /// iterate through all TEB states and add/remove states!
  bool modified = true;

  for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
  {
    modified = false;

    for(int i=0; i < sizeTimeDiffs(); ++i) // TimeDiff connects Point(i) with Point(i+1)
    {
      if(TimeDiff(i) > dt_ref + dt_hysteresis && sizeTimeDiffs()<max_samples)
      {
        //RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"), "teb_local_planner: autoResize() inserting new bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());

        double newtime = 0.5*TimeDiff(i);

        TimeDiff(i) = newtime;
        insertPose(i+1, PoseSE2::average(Pose(i),Pose(i+1)) );
        insertTimeDiff(i+1,newtime);

        modified = true;
      }
      else if(TimeDiff(i) < dt_ref - dt_hysteresis && sizeTimeDiffs()>min_samples) // only remove samples if size is larger than min_samples.
      {
        //RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"), "teb_local_planner: autoResize() deleting bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());

        if(i < ((int)sizeTimeDiffs()-1))
        {
          TimeDiff(i+1) = TimeDiff(i+1) + TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i+1);
        }
        else
        { // last motion should be adjusted, shift time to the interval before
          TimeDiff(i-1) += TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
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

double TimedElasticBand::getSumOfTimeDiffsUpToIdx(int index) const
{
  assert(index<=timediff_vec_.size());

  double time = 0;

  for(int i = 0; i < index; ++i)
  {
    time += timediff_vec_.at(i)->dt();
  }

  return time;
}

double TimedElasticBand::getAccumulatedDistance() const
{
  double dist = 0;

  for(int i=1; i<sizePoses(); ++i)
  {
      dist += (Pose(i).position() - Pose(i-1).position()).norm();
  }
  return dist;
}

bool TimedElasticBand::initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double max_vel_x, int min_samples, bool guess_backwards_motion)
{
  if (!isInit())
  {
    addPose(start); // add starting point
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization

    double timestep = 0.1;

    if (diststep!=0)
    {
      Eigen::Vector2d point_to_goal = goal.position()-start.position();
      double dir_to_goal = std::atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
      double dx = diststep*std::cos(dir_to_goal);
      double dy = diststep*std::sin(dir_to_goal);
      double orient_init = dir_to_goal;
      // check if the goal is behind the start pose (w.r.t. start orientation)
      if (guess_backwards_motion && point_to_goal.dot(start.orientationUnitVec()) < 0)
        orient_init = g2o::normalize_theta(orient_init+M_PI);
      // TODO: timestep ~ max_vel_x_backwards for backwards motions

      double dist_to_goal = point_to_goal.norm();
      double no_steps_d = dist_to_goal/std::abs(diststep); // ignore negative values
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d);

      if (max_vel_x > 0) timestep = diststep / max_vel_x;

      for (unsigned int i=1; i<=no_steps; i++) // start with 1! starting point had index 0
      {
        if (i==no_steps && no_steps_d==(float) no_steps)
            break; // if last conf (depending on stepsize) is equal to goal conf -> leave loop
        addPoseAndTimeDiff(start.x()+i*dx,start.y()+i*dy,orient_init,timestep);
      }

    }

    // if number of samples is not larger than min_samples, insert manually
    if ( sizePoses() < min_samples-1 )
    {
      RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"),
                   "initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        if (max_vel_x > 0) timestep = (intermediate_pose.position()-BackPose().position()).norm()/max_vel_x;
        addPoseAndTimeDiff( intermediate_pose, timestep ); // let the optimier correct the timestep (TODO: better initialization
      }
    }

    // add goal
    if (max_vel_x > 0) timestep = (goal.position()-BackPose().position()).norm()/max_vel_x;
    addPoseAndTimeDiff(goal,timestep); // add goal point
    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
  }
  else // size!=0
  {
    RCLCPP_WARN(rclcpp::get_logger("teb_local_planner"),
                 "Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    RCLCPP_WARN(rclcpp::get_logger("teb_local_planner"),
                 "Number of TEB configurations: %d, Number of TEB timediffs: %d",(unsigned int) sizePoses(),(unsigned int) sizeTimeDiffs());
    return false;
  }
  return true;
}


bool TimedElasticBand::initTrajectoryToGoal(const std::vector<geometry_msgs::msg::PoseStamped>& plan, double max_vel_x, double max_vel_theta, bool estimate_orient, int min_samples, bool guess_backwards_motion)
{

  if (!isInit())
  {
    PoseSE2 start(plan.front().pose);
    PoseSE2 goal(plan.back().pose);

    addPose(start); // add starting point with given orientation
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization

    bool backwards = false;
    if (guess_backwards_motion && (goal.position()-start.position()).dot(start.orientationUnitVec()) < 0) // check if the goal is behind the start pose (w.r.t. start orientation)
        backwards = true;
    // TODO: dt ~ max_vel_x_backwards for backwards motions

    for (int i=1; i<(int)plan.size()-1; ++i)
    {
        double yaw;
        if (estimate_orient)
        {
            // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
            double dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
            double dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
            yaw = std::atan2(dy,dx);
            if (backwards)
                yaw = g2o::normalize_theta(yaw+M_PI);
        }
        else
        {
            yaw = tf2::getYaw(plan[i].pose.orientation);
        }
        PoseSE2 intermediate_pose(plan[i].pose.position.x, plan[i].pose.position.y, yaw);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, max_vel_x, max_vel_theta);
        addPoseAndTimeDiff(intermediate_pose, dt);
    }

    // if number of samples is not larger than min_samples, insert manually
    if ( sizePoses() < min_samples-1 )
    {
      RCLCPP_DEBUG(rclcpp::get_logger("teb_local_planner"),
                   "initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        double dt = estimateDeltaT(BackPose(), intermediate_pose, max_vel_x, max_vel_theta);
        addPoseAndTimeDiff( intermediate_pose, dt ); // let the optimier correct the timestep (TODO: better initialization
      }
    }

    // Now add final state with given orientation
    double dt = estimateDeltaT(BackPose(), goal, max_vel_x, max_vel_theta);
    addPoseAndTimeDiff(goal, dt);
    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
  }
  else // size!=0
  {
    RCLCPP_WARN(rclcpp::get_logger("teb_local_planner"),
                 "Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    RCLCPP_WARN(rclcpp::get_logger("teb_local_planner"),
                 "Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
    return false;
  }

  return true;
}


int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance, int begin_idx) const
{
  int n = sizePoses();
  if (begin_idx < 0 || begin_idx >= n)
    return -1;

  double min_dist_sq = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = begin_idx; i < n; i++)
  {
    double dist_sq = (ref_point - Pose(i).position()).squaredNorm();
    if (dist_sq < min_dist_sq)
    {
      min_dist_sq = dist_sq;
      min_idx = i;
    }
  }

  if (distance)
    *distance = std::sqrt(min_dist_sq);

  return min_idx;
}


int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance) const
{
  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double dist = distance_point_to_segment_2d(point, ref_line_start, ref_line_end);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_idx = i;
    }
  }

  if (distance)
    *distance = min_dist;
  return min_idx;
}

int TimedElasticBand::findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance) const
{
  if (vertices.empty())
    return 0;
  else if (vertices.size() == 1)
    return findClosestTrajectoryPose(vertices.front());
  else if (vertices.size() == 2)
    return findClosestTrajectoryPose(vertices.front(), vertices.back());

  double min_dist = std::numeric_limits<double>::max();
  int min_idx = -1;

  for (int i = 0; i < sizePoses(); i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double dist_to_polygon = std::numeric_limits<double>::max();
    for (int j = 0; j < (int) vertices.size()-1; ++j)
    {
      dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, vertices[j], vertices[j+1]));
    }
    dist_to_polygon = std::min(dist_to_polygon, distance_point_to_segment_2d(point, vertices.back(), vertices.front()));
    if (dist_to_polygon < min_dist)
    {
      min_dist = dist_to_polygon;
      min_idx = i;
    }
  }

  if (distance)
    *distance = min_dist;

  return min_idx;
}


int TimedElasticBand::findClosestTrajectoryPose(const Obstacle& obstacle, double* distance) const
{
  const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
  if (pobst)
    return findClosestTrajectoryPose(pobst->position(), distance);

  const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
  if (lobst)
    return findClosestTrajectoryPose(lobst->start(), lobst->end(), distance);

  const PolygonObstacle* polyobst = dynamic_cast<const PolygonObstacle*>(&obstacle);
  if (polyobst)
    return findClosestTrajectoryPose(polyobst->vertices(), distance);

  return findClosestTrajectoryPose(obstacle.getCentroid(), distance);
}


void TimedElasticBand::updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  if (new_start && sizePoses()>0)
  {
    // find nearest state (using l2-norm) in order to prune the trajectory
    // (remove already passed states)
    double dist_cache = (new_start->position()- Pose(0).position()).norm();
    double dist;
    int lookahead = std::min<int>( sizePoses()-min_samples, 10); // satisfy min_samples, otherwise max 10 samples

    int nearest_idx = 0;
    for (int i = 1; i<=lookahead; ++i)
    {
      dist = (new_start->position()- Pose(i).position()).norm();
      if (dist<dist_cache)
      {
        dist_cache = dist;
        nearest_idx = i;
      }
      else break;
    }

    // prune trajectory at the beginning (and extrapolate sequences at the end if the horizon is fixed)
    if (nearest_idx>0)
    {
      // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
      // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since Pose(0) is fixed during optimization!
      deletePoses(1, nearest_idx);  // delete first states such that the closest state is the new first one
      deleteTimeDiffs(1, nearest_idx); // delete corresponding time differences
    }

    // update start
    Pose(0) = *new_start;
  }

  if (new_goal && sizePoses()>0)
  {
    BackPose() = *new_goal;
  }
};


bool TimedElasticBand::isTrajectoryInsideRegion(double radius, double max_dist_behind_robot, int skip_poses)
{
    if (sizePoses()<=0)
        return true;

    double radius_sq = radius*radius;
    double max_dist_behind_robot_sq = max_dist_behind_robot*max_dist_behind_robot;
    Eigen::Vector2d robot_orient = Pose(0).orientationUnitVec();

    for (int i=1; i<sizePoses(); i=i+skip_poses+1)
    {
        Eigen::Vector2d dist_vec = Pose(i).position()-Pose(0).position();
        double dist_sq = dist_vec.squaredNorm();

        if (dist_sq > radius_sq)
        {
            RCLCPP_INFO(rclcpp::get_logger("teb_local_planner"), "outside robot");
            return false;
        }

        // check behind the robot with a different distance, if specified (or >=0)
        if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 && dist_sq > max_dist_behind_robot_sq)
        {
            RCLCPP_INFO(rclcpp::get_logger("teb_local_planner"), "outside robot behind");
            return false;
        }

    }
    return true;
}




} // namespace teb_local_planner

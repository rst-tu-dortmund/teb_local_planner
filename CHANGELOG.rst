^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teb_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2018-08-14)
------------------
* bugfix in calculateHSignature. Fixes `#90 <https://github.com/rst-tu-dortmund/teb_local_planner/issues/90>`_.
* fixed centroid computation in a special case of polygon-obstacles
* Contributors: Christoph Rösmann

0.8.0 (2018-08-06)
------------------
* First melodic release
* Updated to new g2o API
* Migration to tf2
* Contributors: Christoph Rösmann

0.7.3 (2018-07-05)
------------------
* Parameter `switching_blocking_period` added to homotopy class planner parameter group.
  Values greater than zero enforce the homotopy class planner to only switch to new equivalence classes as soon
  as the given period is expired (this might reduce oscillations in some scenarios). The value is set to zero seconds
  by default in order to not change the behavior of existing configurations.
* Fixed unconsistent naming of parameter `global_plan_viapoint_sep`.
  The parameter retrieved at startup was `global_plan_via_point_sep` and via dynamic_reconfigure it was `global_plan_viapoint_sep`.
  `global_plan_via_point_sep` has now been replaced by `global_plan_viapoint_sep` since this is more consistent with the variable name
  in the code as well as `weight_viapoint` and the ros wiki description.
  In order to not break things, the old parameter name can still be used. However, a deprecated warning is printed.
* transformGlobalPlan searches now for the closest point within the complete subset of the global plan in the local costmap:
  In every sampling interval, the global plan is processed in order to find the closest pose to the robot (as reference start) 
  and the current end pose (either local at costmap boundary or max_global_plan_lookahead_dist).
  Previously, the search algorithm stopped as soon as the distance to the robot increased once. 
  This caused troubles with more complex global plans, hence the new strategy checks the complete subset
  of the global plan in the local costmap for the closest distance to the robot.
* via-points that are very close to the current robot pose or behind the robot are now skipped (in non-ordered mode)
* Edge creation: minor performance improvement for dynamic obstacle edges
* dynamic_reconfigure: parameter visualize_with_time_as_z_axis_scale moved to group trajectory
* Contributors: Christoph Rösmann

0.7.2 (2018-06-08)
------------------
* Adds the possibility to provide via-points via a topic. 
  Currently, the user needs to decide whether to receive via-points from topic or to obtain them from the global reference plan 
  (e.g., activate the latter by setting global_plan_viapoint_sep>0 as before).
  A small test script publish_viapoints.py is provided to demonstrate the feature within test_optim_node.
* Contributors: Christoph Rösmann

0.7.1 (2018-06-05)
------------------
* Fixed a crucial bug (from 0.6.6): A cost function for prefering a clockwise resp. anti-clockwise turn was enabled by default.
  This cost function was only intended to be active only for recovering from an oscillating robot. 
  This cost led to a penalty for one of the turning directions and hence the maximum turning rate for the penalized direction could not be reached.
  Furthermore, which is more crucial: since the penalty applied only to a small (initial) subset of the trajectory, the overall control performance was poor
  (huge gap between planned motion and closed-loop trajectories led to frequent corrections of the robot pose and hence many motion reversals).
* Adds support for circular obstacle types. This includes support for the radius field in costmap_converter::ObstacleMsg
* rqt reconfigure: parameters are now grouped in tabs (robot, trajectory, viapoints, ...)
* Update to use non deprecated pluginlib macro
* Python scripts updated to new obstacle message definition.
* Fixed issue when start and end are at the same location (PR #43)
* Normalize marker quaternions in *test_optim_node*
* Contributors: Christoph Rösmann, Alexander Reimann, Mikael Arguedas, wollip

0.7.0 (2017-09-23)
------------------
* This update introduces support for dynamic obstacles (thanks to Franz Albers, who implemented and tested the code).
  Dynamic obstacle support requires parameter *include\_dynamic\_obstacles* to be activated.
  Note, this feature is still experimental and subject to testing.
  Motion prediction is performed using a constant velocity model.
  Dynamic obstacles might be incorporated as follows:
  * via a custom message provided on topic ~/obstacles (warning: we changed the message type from teb_local_planner/ObstacleMsg to costmap_converter/ObstacleArrayMsg).
  * via the CostmapToDynamicObstacles plugin as part of the costmap\_converter package (still experimental).
  A tutorial is going to be provided soon.
* FeedbackMsg includes a ObstacleMsg instead of a polygon
* ObstacleMsg removed from package since it is now part of the costmap\_converter package.
* Homotopy class planer code update: graph search methods and equivalence classes (h-signatures) are now 
  implemented as subclasses of more general interfaces.
* TEB trajectory initialization now uses a max\_vel\_x argument instead of the desired time difference in order to give the optimizer a better warm start. 
  Old methods are marked as deprecated. This change does not affect users settings.
* Inplace rotations removed from trajectory initialization to improve convergence speed of the optimizer
* teb\_local\_planner::ObstacleMsg removed in favor of costmap\_converter::ObstacleArrayMsg. This also requires custom obstacle publishers to update to the new format
* the "new" trajectory resizing method is only activated, if "include_dynamic_obstacles" is set to true.
  We introduced the non-fast mode with the support of dynamic obstacles
  (which leads to better results in terms of x-y-t homotopy planning).
  However, we have not yet tested this mode intensively, so we keep
  the previous mode as default until we finish our tests.
* added parameter and code to update costmap footprint if it is dynamic (#49)
* Contributors: Franz Albers, Christoph Rösmann, procopiostein

0.6.6 (2016-12-23)
------------------
* Strategy for recovering from oscillating local plans added (see new parameters)
* Horizon reduction for resolving infeasible trajectories is not activated anymore if the global goal is already selected
  (to avoid oscillations due to changing final orientations)
* Global plan orientations are now taken for TEB initialization if lobal_plan_overwrite_orientation==true
* Parameter max_samples added
* Further fixes (thanks to Matthias Füller and Daniel Neumann for providing patches)

0.6.5 (2016-11-15)
------------------
* The trajectory is now initialized backwards for goals close to and behind the robot.
  Parameter 'allow_init_with_backwards_motion' added.
* Updated the TEB selection in the HomotopyClassPlanner.
  * A new parameter is introduced to prefer the equivalence class of the initial plan
  * Fixed some bugs related to the deletion of candidates and for keeping the equivalence class of the initial plan.
* Weight adaptation added for obstacles edges.
  Added parameter 'weight_adapt_factor'.
  Obstacle weights are repeatedly scaled by this factor in each outer TEB iteration.
  Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions.
* Added a warning if the optim footprint + min_obstacle_dist is smaller than the costmap footprint.
  Validation is performed by only comparing the inscribed radii of the footprints.
* Revision/extension of the reduced-horizon backup mode which is triggered in case infeasible trajectories are detected.
* Changed HSignature to a generic equivalence class
* Minor changes

0.6.4 (2016-10-23)
------------------
* New default obstacle association strategy:
  During optimization graph creation, for each pose of the trajectory a
  relevance detection is performed before considering the obstacle
  during optimization. New parameters are introduced. The
  old strategy is kept as 'legacy' strategy (see parameters).
* Computation of velocities, acceleration and turning radii extended:
  Added an option to compute the actual arc length
  instead of using the Euclidean distance approximation (see parameter `exact_arc_length`.
* Added intermediate edge layer for unary, binary and multi edges in order to reduce code redundancy.
* Script for visualizing velocity profile updated to accept the feedback topic name via rosparam server
* Removed TebConfig dependency in TebVisualization
* PolygonObstacle can now be constructed using a vertices container
* HomotopyClassPlanner public interface extended
* Changed H-Signature computation to work 'again' with few obstacles such like 1 or 2
* Removed inline flags in visualization.cpp
* Removed inline flags in timed_elastic_band.cpp.
  Fixes `#15 <https://github.com/rst-tu-dortmund/teb_local_planner/issues/15>`_.
* Increased bounds of many variables in dynamic_reconfigure. 
  Resolves `#14 <https://github.com/rst-tu-dortmund/teb_local_planner/issues/14>`_.
  The particular variables are maximum velocities, maximum accelerations,
  minimum turning radius,...
  Note: optimization weights and dt_ref as well as dt_hyst are not
  tuned for velocities and accelerations beyond
  the default values (e.g. >1 m/s). Just increasing the maximum velocity
  bounds without adjusting the other parameters leads to an insufficient behavior.
* Default parameter value update: 'costmap_obstacles_behind_robot_dist'
* Additional minor fixes.

0.6.3 (2016-08-17)
------------------
* Changed the f0 function for calculating the H-Signature.
  The new one seems to be more robust for a much larger number of obstacles
  after some testing.
* HomotopyClassPlanner: vertex collision check removed since collisions will be determined in the edge collision check again
* Fixed distance calculation polygon-to-polygon-obstacle
* cmake config exports now *include directories* of external packages for dependent projects
* Enlarged upper bounds on goal position and orientation tolerances in *dynamic_reconfigure*. Fixes #13.


0.6.2 (2016-06-15)
------------------
* Fixed bug causing the goal to disappear in case the robot arrives with non-zero orientation error.
* Inflation mode for obstacles added.
* The homotopy class of the global plan is now always forced to be initialized as trajectory.
* The initial velocity of the robot is now taken into account correctly for
  all candidate trajectories.
* Removed a check in which the last remaining candidate trajectory was rejected if it was close to an obstacle.
  This fix addresses issue `#7 <https://github.com/rst-tu-dortmund/teb_local_planner/issues/7>`_

0.6.1 (2016-05-23)
------------------
* Debian ARM64 library path added to SuiteSparse cmake find-script (resolves ARM compilation issue)


0.6.0 (2016-05-22)
------------------
* Extended support to holonomic robots
* Wrong parameter namespace for *costmap_converter* plugins fixed
* Added the option to scale the length of the hcp sampling area
* Compiler warnings fixed.
* Workaround for compilation issues that are caused by a bug in boost 1.58
  concerning the graph library (missing move constructor/assignment operator
  in boost source).
* Using *tf_listener* from *move_base* now.
* Via-point support improved.
  Added the possibility to take the actual order of via-points into account.
  Additionally, via-points beyond start and goal are now included.
* Obsolete include of the angles package header removed
* Update to package.xml version 2
* Some other minor fixes.


0.4.0 (2016-04-19)
------------------
* The teb_local_planner supports a path-following mode (w.r.t. the global plan) and via-points now.
  This allows the user to adapt the tradeoff between time-optimality and path-following.
  Check out the new tutorial: "Following the Global Plan (Via-Points)".
* All external configuration and launch files are removed, since they are part
  of the new teb_local_planner_tutorials package.


0.3.1 (2016-04-14)
------------------
* Fixed wrong coordinate transformation in 'line' and 'polygon' footprint models.
* Trajectory selection strategy in case of multiple topologies updated:
  * The obstacle costs for selection can now be scaling separately.
  * The cost regarding time optimality can now be replaced by the actual transition time.
  * Added a hysteresis to cost comparison between a new and the previously selected trajectory.
  * In the default parameter setting the strategy is similar to release 0.3.0.
* Warning message removed that occured if an odom message with only zeros was received.


0.3.0 (2016-04-08)
------------------
* Different/custom robot footprints are now supported and subject to optimization (refer to the new tutorial!).
* The new robot footprint is also visualized using the common marker topic.
* The strategy of taking occupied costmap cells behind the robot into account has been improved.
  These changes significantly improve navigation close to walls.
* Parameter 'max_global_plan_lookahead_dist' added.
  Previously, the complete subset of the global plan contained in the local costmap
  was taken into account for choosing the current intermediate goal point. With this parameter, the maximum
  length of the reference global plan can be limited. The actual global plan subset
  is now computed using the logical conjunction of both local costmap size and 'max_global_plan_lookahead_dist'.
* Bug fixes:
  * Fixed a compilation issue on ARM architectures
  * If custom obstacles are used, the container with old obstacles is now cleared properly. 
* Parameter cleanup: 
  * "weight_X_obstacle" parameters combined to single parameter "weight_obstacle".
  * "X_obstacle_poses_affected" parameters combined to single parameter "obstacle_poses_affected". 
  * Deprecated parameter 'costmap_emergency_stop_dist' removed.
* Code cleanup


0.2.3 (2016-02-01)
------------------
* Marker lifetime changed
* In case the local planner detects an infeasible trajectory it does now try to
  reduce the horizon to 50 percent of the length. The trajectory is only reduced
  if some predefined cases are detected.
  This mechanism constitutes a backup behavior.
* Improved carlike robot support.
  Instead of commanding the robot using translational and rotational velocities,
  the robot might also be commanded using the transl. velocity and steering angle.
  Appropriate parameters are added to the config.
* Changed default parameter for 'h_signature_threshold' from 0.01 to 0.1 to better match the actual precision.
* Some python scripts for data conversion added
* Minor other changes

0.2.2 (2016-01-11)
------------------
* Carlike robots (ackermann steering) are supported from now on (at least experimentally) 
  by specifying a minimum bound on the turning radius.
  Currently, the output of the planner in carlike mode is still (v,omega).
  Since I don't have any real carlike robot, I would be really happy if someone could provide me with
  some feedback to further improve/extend the support.
* Obstacle cost function modified to avoid undesired jerks in the trajectory.
* Added a feedback message that contains current trajectory information (poses, velocities and temporal information).
  This is useful for analyzing and debugging the velocity profile e.g. at runtime.
  The message will be published only if it's activated (rosparam).
  A small python script is added to plot the velocity profile (while *test_optim_node* runs).
* Cost functions are now taking the direction/sign of the translational velocity into account:
  Specifying a maximum backwards velocity other than forward velocity works now.
  Additionally, the change in acceleration is now computed correctly if the robot switches directions.
* The global plan is now pruned such that already passed posses are cut off
  (relevant for global planners with *planning_rate=0*).
* Fixed issue#1: If a global planner with *planning_rate=0* was used, 
  a TF timing/extrapolation issue appeared after some time.
* The planner resets now properly if the velocity command cannot be computed due to invalid optimization results.


0.2.1 (2015-12-30)
------------------
* This is an important bugfix release.
* Fixed a major issue concerning the stability and performance of the optimization process. Each time the global planner was updating the global plan, the local planner was resetted completely even if
  the updated global plan did not differ from the previous one. This led to stupid reinitializations and a slighly jerky behavior if the update rate of the global planner was high (each 0.5-2s).
  From now on the local planner is able to utilize the global plan as a warm start and determine automatically whether to reinitialize or not.
* Support for polygon obstacles extended and improved (e.g. the homotopy class planner does now compute actual distances to the polygon rather than utilizing the distance to the centroid).

0.2.0 (2015-12-23)
------------------
* The teb_local_planner supports costmap_converter plugins (pluginlib) from now on. Those plugins convert occupied costmap2d cells into polygon shapes.
  The costmap_converter is disabled by default, since the extension still needs to be tested (parameter choices, computation time advantages, etc.). 
  A tutorial will explain how to activate the converter using the ros-param server.

0.1.11 (2015-12-12)
-------------------
* This is a bugfix release (it fixes a lot of issues which occured frequently when the robot was close to the goal)

0.1.10 (2015-08-13)
-------------------
* The optimizer copies the global plan as initialization now instead of using a simple straight line approximation.
* Some bugfixes and improvements

0.1.9 (2015-06-24)
------------------
* Fixed a segmentation fault issue. This minor update is crucial for stability.

0.1.8 (2015-06-08)
------------------
* Custom obstacles can be included via publishing dedicated messages
* Goal-reached-condition also checks orientation error (desired yaw) now
* Numerical improvements of the h-signature calculation
* Minor bugfixes

0.1.7 (2015-05-22)
------------------
* Finally fixed saucy compilation issue by retaining compatiblity to newer distros
  (my "new" 13.10 VM helps me to stop spamming new releases for testing).

0.1.6 (2015-05-22)
------------------
* Fixed compilation errors on ubuntu saucy caused by different FindEigen.cmake scripts.
  I am not able to test releasing on saucy, forcing me to release again and again. Sorry.

0.1.5 (2015-05-21)
------------------
* Added possibility to dynamically change parameters of test_optim_node using dynamic reconfigure.
* Fixed a wrong default-min-max tuple in the dynamic reconfigure config.
* Useful config and launch files are now added to cmake install.
* Added install target for the test_optim_node executable.

0.1.4 (2015-05-20)
------------------
* Fixed compilation errors on ROS Jade

0.1.3 (2015-05-20)
------------------
* Fixed compilation errors on ubuntu saucy

0.1.2 (2015-05-19)
------------------
* Removed unused include that could break compilation.

0.1.1 (2015-05-19)
------------------
* All files added to the indigo-devel branch
* Initial commit
* Contributors: Christoph Rösmann

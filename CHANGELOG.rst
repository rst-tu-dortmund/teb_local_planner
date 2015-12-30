^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teb_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2015-12-30)
------------------
* This is an important bugfix release.
* Fixed a major issue concerning the stability and performance of the optimization process. Each time the global planner was updating the global plan, the local planner was resetted completly even if
  the updated global plan did not differ from the previous one. This led to stupid reinitializations and a slighly jerky behavior if the update rate of the global planner was high (each 0.5-2s).
  From now on the local planner is able to utilize the global plan as a warm start and determine automatically whether to reinitialize or not.
* Support for polyon obstacles extended and improved (e.g. the homotopy class planner does now compute actual distances to the polygon rather than utilizing the distance to the centroid).

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

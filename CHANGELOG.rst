^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teb_local_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2015-06-08)
------------------
* Custom obstacles can included via publishing dedicated messages
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

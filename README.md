teb_local_planner ROS Package
=============================

The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, 
separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials.

Build status of the *melodic-devel* branch:
- ROS Buildfarm (Melodic): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)

### Port to ROS2
This branch is the teb_local_planner package ported to ROS2(Dashing Diademata). Currently, it is currently compatible with [Navigation2(master branch)](https://github.com/ros-planning/navigation2/tree/master)([226f06c](https://github.com/ros-planning/navigation2/commit/226f06ce282c727ca240ce8be0cb4b093e26343b)). You can test teb_local_planner with Navigation2 and TurtleBot3 simulation by launching the following command.
```
ros2 launch teb_local_planner teb_tb3_simulation_launch.py
```

## Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the planner for your own research:*

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.

<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/lato-black.png" alt="Buy Me A Coffee" height="31px" width="132px" ></a>

## Videos

The left of the following videos presents features of the package and shows examples from simulation and real robot situations.
Some spoken explanations are included in the audio track of the video. 
The right one demonstrates features introduced in version 0.2 (supporting car-like robots and costmap conversion). Please watch the left one first.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" 
alt="teb_local_planner - An Optimal Trajectory Planner for Mobile Robots" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" 
alt="teb_local_planner - Car-like Robots and Costmap Conversion" width="240" height="180" border="10" /></a>

## License

The *teb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+, 
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com. 
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner



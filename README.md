teb_local_planner ROS Package
=============================

The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, 
separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials.

Build status of the *master* branch:
- ROS Jade: [![Jade Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-jade-teb_local_planner)](http://jenkins.ros.org/job/devel-jade-teb_local_planner/)
- ROS Indigo: [![Indigo Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-teb_local_planner)](http://jenkins.ros.org/job/devel-indigo-teb_local_planner/)


### Papers Describing the Approach

- Rösmann C., Feiten W., Wösch T., Hoffmann F. and Bertram. T.: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, 2012, pp 74–79.
- Rösmann C., Feiten W., Wösch T., Hoffmann F. and Bertram. T.: Efficient trajectory optimization using a sparse model. Proc. European Conference on Mobile Robots, Spain, Barcelona, 2013, pp. 138–143. 


### License

The *teb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+, 
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com. 
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

### Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner



# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This is all-in-one launch script intended for use by nav2 developers.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the launch directory
    teb_launch_dir = os.path.join(
        get_package_share_directory('teb_local_planner'), 'launch')
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    
    params_file = os.path.join(teb_launch_dir, 'teb_params.yaml')
    rviz_config_file = os.path.join(teb_launch_dir, 'teb_view.rviz')
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'tb3_simulation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'rviz_config_file': rviz_config_file}.items())

    ld = LaunchDescription()
    ld.add_action(bringup_cmd)

    return ld

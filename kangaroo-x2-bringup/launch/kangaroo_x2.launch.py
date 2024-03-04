# Copyright 2022 Walter Lucetti
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
###########################################################################

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    node_name = LaunchConfiguration('node_name')

    # Kangaroo x2 node configuration file
    def_kx2_config_path = os.path.join(
        get_package_share_directory('kangaroo_x2_bringup'),
        'params',
        'default_params.yaml'
    )

    # Launch arguments
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='kangaroo_x2',
        description='Name of the node'
    )

    declare_cfg_path_cmd = DeclareLaunchArgument(
        'cfg_path',
        default_value=def_kx2_config_path,
        description='Custom configuration file'
    )

    # Kangaroo x2 lifecycle component node
    kx2_lc_component = ComposableNode(
        package='kangaroo_x2_component',
        namespace='',
        plugin='kx2::KangarooX2Component',
        name=node_name,
        parameters=[def_kx2_config_path],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Composition Container
    container = ComposableNodeContainer(
        name='kx2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                kx2_lc_component],
        output='screen',
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_node_name_cmd)
    ld.add_action(declare_cfg_path_cmd)

    # Composition Container
    ld.add_action(container)

    return ld

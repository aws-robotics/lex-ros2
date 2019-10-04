# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
# You may not use this file except in compliance with the License.
# A copy of the License is located at
#
#  http://aws.amazon.com/apache2.0
#
# or in the "license" file accompanying this file. This file is distributed
# on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing
# permissions and limitations under the License.

"""Launch a lifecycle lex node"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('lex_node'),
                                        'config', 'sample_configuration.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            name='node_name',
            default_value='lex_node'
        ),
        DeclareLaunchArgument(
            name='config_file',
            default_value=config_file_path
        ),
        Node(
            package='lex_node',
            node_executable='lex_node',
            node_name=LaunchConfiguration('node_name'),
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        ),
    ])

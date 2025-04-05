#incluir launchers de yolobringup, tfseeker, tfpublisher, yolo_parser_multiperson
# Copyright 2024 Intelligent Robotics Lab
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Obtener rutas de los paquetes
    follow_person_pkg = get_package_share_directory('follow_person')
    yolo_pkg = get_package_share_directory('yolo_bringup')

    # Archivo de parámetros
    param_file = os.path.join(follow_person_pkg, 'config', 'params.yaml')

    # Nodo principal
    parser_cmd = Node(
        package='yolo_p7',
        executable='yolo_parser',
        output='screen',
        parameters=[param_file],
    )
    tf_publisher_cmd = Node(
        package='follow_person',
        executable='tf_publisher',
        output='screen',
        parameters=[param_file],
    )
    tf_seeker_cmd = Node(
        package='follow_person',
        executable='seeker',
        output='screen',
        parameters=[param_file],
    )


    # Incluir el launcher de laser_vff
    yolo_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                yolo_pkg,
                'launch',
                'yolo.sim.launch.py'
            )
        )
    )


    # Crear la descripción del launch
    ld = LaunchDescription()
    ld.add_action(yolo_3d_launch)
    ld.add_action(tf_publisher_cmd)
    ld.add_action(tf_seeker_cmd)
    ld.add_action(parser_cmd)

    return ld

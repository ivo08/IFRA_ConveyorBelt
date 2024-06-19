#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Dr. Seemal Asif  - s.asif@cranfield.ac.uk                                   #
#           Prof. Phil Webb  - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: June, 2023.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA-Cranfield (2023) Gazebo-ROS2 Conveyor Belt Plugin. URL: https://github.com/IFRA-Cranfield/IFRA_ConveyorBelt.

# conveyorbelt.launch.py:
# Launch file for the IFRA_ConveyorBelt GAZEBO SIMULATION in ROS2:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
import xacro
import yaml

# LOAD FILE:


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #


def generate_launch_description():

    # ***** GAZEBO ***** #
    # DECLARE Gazebo WORLD file:
    conveyorbelt_gazebo_path = os.path.join(
        get_package_share_directory('conveyorbelt_gazebo'),
        'worlds',
        'conveyorbelt.world')

    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': conveyorbelt_gazebo_path}.items(),
    )

    # DECLARE Gazebo ROS2 service: ros2 service call /b2/CONVEYORPOWER conveyorbelt_msgs/srv/ConveyorBeltControl "{power: 5}"
    service_belt_1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/b1/CONVEYORPOWER',
             'conveyorbelt_msgs/srv/ConveyorBeltControl', '{power: 10}'],
        output='screen'
    )
    service_belt_2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/b2/CONVEYORPOWER',
             'conveyorbelt_msgs/srv/ConveyorBeltControl', '{power: 10}'],
        output='screen'
    )
    service_belt_3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/b3/CONVEYORPOWER',
             'conveyorbelt_msgs/srv/ConveyorBeltControl', '{power: 10}'],
        output='screen'
    )

    # DECLARE Gazebo ROS2 run: ros2 run ros2_conveyorbelt SpawnObject.py
    spawn_object = ExecuteProcess(
        cmd=['ros2', 'run', 'ros2_conveyorbelt', 'SpawnObject.py'],
        output='screen'
    )

    # DECLARE a ros2 run with argumentos to run the defect_detector_node
    defect_detector_node_1 = ExecuteProcess(
        cmd=['ros2', 'run', 'defect_detector', 'defect_detector_node', '--prefix', 'b1'],
        output='screen'
    )

    defect_detector_node_2 = ExecuteProcess(
        cmd=['ros2', 'run', 'defect_detector', 'defect_detector_node', '--prefix', 'b2'],
        output='screen'
    )

    defect_detector_node_3 = ExecuteProcess(
        cmd=['ros2', 'run', 'defect_detector', 'defect_detector_node', '--prefix', 'b3'],
        output='screen'
    )


    # Call a different launch file with namespace
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gripper_controller_python'), 'launch'), '/gripper_position_control.launch.py'])
    )

    """ gripper_launch_namespace = GroupAction([
        PushRosNamespace(namespace='gripper1'),
        gripper_launch,
    ]) """

    # Launch file from rv2
    rv2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rv2'), 'launch'), '/rv2.launch.py'])
    )


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo,
        service_belt_1, service_belt_2, service_belt_3,
        gripper_launch,
        spawn_object,
        defect_detector_node_1, defect_detector_node_2, defect_detector_node_3,
        #gripper_launch_namespace
        rv2_launch
    ])

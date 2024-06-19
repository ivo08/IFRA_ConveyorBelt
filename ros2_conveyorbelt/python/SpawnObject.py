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

# ===================================== COPYRIGHT ===================================== #
# SpawnObject.py script taken from:
# IFRA-Cranfield (2023) ROS 2 Sim-to-Real Robot Control. URL: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl -> ros2srrc_execution ROS2 Package.

# IMPORT LIBRARIES:
import argparse
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import rclpy
import random
from time import sleep

# Reference to SPAWN OBJECT (.urdf or .xacro file) from the terminal shell:
# REFERENCE: ros2 run ros2_conveyorbelt SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}
# EXAMPLE: BOX -> ros2 run ros2_conveyorbelt SpawnObject.py --package "conveyorbelt_gazebo" --urdf "box.urdf" --name "box" --x 0.0 --y -0.5 --z 0.76


# Coordinates where the box can be spawned
POSSIBLE_COORDINATES = (
    (2.1, -0.5, 0.77), (1.1, -0.5, 0.77), (0.1, -0.5, 0.77))


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(
        description='Spawn object into our Gazebo world.')
    parser.add_argument('--package', type=str, default='conveyorbelt_gazebo',
                        help='Package where URDF/XACRO file is located.')
    # parser.add_argument('--urdf', type=str, default='', help='URDF of the object to spawn.')
    # parser.add_argument('--name', type=str, default='OBJECT', help='Name of the object to spawn.')
    parser.add_argument('--namespace', type=str, default='ros2Grasp',
                        help='ROS namespace to apply to the tf and plugins.')
    parser.add_argument('--ns', type=bool, default=True,
                        help='Whether to enable namespacing')

    from collections import deque
    import time

    # Create a queue to keep track of the created entities and their creation time
    created_entities = deque()

    # Start node:
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    try:
        while True:
            for i in range(len(POSSIBLE_COORDINATES)):
                # Get the coordinates
                x, y, z = POSSIBLE_COORDINATES[i]

                # Create na random hash for name
                name = str(random.getrandbits(128))

                args, unknown = parser.parse_known_args()

                # Create a client for the delete_entity service
                delete_client = node.create_client(
                    DeleteEntity, '/delete_entity')

                """ node.get_logger().info(
                    'Creating Service client to connect to `/spawn_entity`') """
                client = node.create_client(SpawnEntity, '/spawn_entity')

                #node.get_logger().info('Connecting to `/spawn_entity` service...')
                if not client.service_is_ready():
                    client.wait_for_service()
                    node.get_logger().info('...connected!')

                # Set data for request:
                request = SpawnEntity.Request()
                request.name = name

                if random.random() > 0.5:
                    urdf_file_path = os.path.join(get_package_share_directory(
                        args.package), 'urdf', 'box_green.urdf')
                else:
                    urdf_file_path = os.path.join(get_package_share_directory(
                        args.package), 'urdf', 'box_red.urdf')

                xacro_file = xacro.process_file(urdf_file_path)
                request.xml = xacro_file.toxml()

                request.initial_pose.position.x = float(x)
                request.initial_pose.position.y = float(y)
                request.initial_pose.position.z = float(z)

                if args.namespace is True:
                    """ node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
                        name, args.namespace, x, y, z)) """

                    request.namespace = args.namespace
                    print(args.namespace)

                else:
                    """ node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
                        name, x, y, z)) """
                    pass

                #node.get_logger().info('Spawning OBJECT using service: `/spawn_entity`')
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future)
                if future.result() is not None:
                    print('response: %r' % future.result())
                else:
                    raise RuntimeError(
                        'exception while calling service: %r' % future.exception())

                # Add the entity to the queue with the current time
                created_entities.append((name, time.time()))

                # Check if the first entity in the queue was created more than 30 seconds ago
                if time.time() - created_entities[0][1] > 20:
                    # If so, delete it
                    name_to_delete, _ = created_entities.popleft()
                    delete_req = DeleteEntity.Request()
                    delete_req.name = name_to_delete
                    future = delete_client.call_async(delete_req)
                    rclpy.spin_until_future_complete(node, future)
                    if future.result() is None:
                        node.get_logger().error('Failed to delete entity %s' % name_to_delete)
            sleep(10)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (Ctrl-C) detected.')
        node.get_logger().info('Shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

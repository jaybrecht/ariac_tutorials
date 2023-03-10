#!/usr/bin/env python3
'''
Example script to move both robots to home position.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 launch ariac_tutorials robot_commander.launch.py
- ros2 run ariac_tutorials move_robots.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    '''
    main function for the move_robots script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    interface.move_robot_home("floor_robot")

    interface.move_robot_home("ceiling_robot")

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

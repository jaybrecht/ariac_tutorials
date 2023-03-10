#!/usr/bin/env python3
'''
Example script to start the competition.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials start_competition.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    '''
    main function for the start_competition script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()
    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials tutorial_1.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

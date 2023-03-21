#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_6.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()

    while rclpy.ok():
        try:
            interface.set_floor_robot_gripper_state(True)
            interface.wait(3)
            interface.set_floor_robot_gripper_state(False)
            interface.wait(3)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

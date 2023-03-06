#!/usr/bin/env python3

import rclpy
from competition_tutorials.competition_interface import CompetitionInterface

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
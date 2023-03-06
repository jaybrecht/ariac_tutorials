#!/usr/bin/env python3

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    interface.move_robot_home("floor_robot")

    interface.move_robot_home("ceiling_robot")

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
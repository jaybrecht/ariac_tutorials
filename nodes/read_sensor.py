#!/usr/bin/env python3

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
            interface.get_logger().info(f'Part Count: {interface.part_count}', throttle_duration_sec=2.0)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

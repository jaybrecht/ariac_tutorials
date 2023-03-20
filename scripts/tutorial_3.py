#!/usr/bin/env python3


import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()
    # The following line enables order displays in the terminal.
    # Set to False to disable.
    interface.parse_incoming_order = True

    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

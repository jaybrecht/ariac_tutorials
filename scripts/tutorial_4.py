#!/usr/bin/env python3
'''
Example script to start the competition.
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_4.py
'''

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

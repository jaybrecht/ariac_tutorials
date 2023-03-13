#!/usr/bin/env python3
'''
Example script to parse, store, and display orders.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials read_orders.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    '''
    main function for the read_orders script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()
    # The following line displays the orders as they are received.
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

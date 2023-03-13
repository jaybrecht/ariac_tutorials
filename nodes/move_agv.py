#!/usr/bin/env python3
'''
Example script to move AGV(s) to assembly stations.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials move_agv.py
'''

import rclpy
from ariac_msgs.msg import Order as OrderMsg
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    '''
    main function for the move_agv script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    while not interface.orders:
        try:
            rclpy.spin_once(interface)
        except KeyboardInterrupt:
            break

    for order in interface.orders:
        if order.order_type == OrderMsg.ASSEMBLY:
            for agv in order.order_task.agv_numbers:
                interface.lock_agv_tray(agv)
                interface.move_agv_to_station(agv, order.order_task.station)

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

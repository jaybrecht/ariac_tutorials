#!/usr/bin/env python3

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

from ariac_msgs.msg import Order, AssemblyTask

def main(args=None):
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    while not interface.orders:
        try:
            rclpy.spin_once(interface)
        except KeyboardInterrupt:
            break

    for order in interface.orders:
        order: Order
        if order.type == Order.ASSEMBLY:
            for agv in order.assembly_task.agv_numbers:
                interface.lock_agv_tray(agv)
                interface.move_agv_to_station(agv, order.assembly_task.station)

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
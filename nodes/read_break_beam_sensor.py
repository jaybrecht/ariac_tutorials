#!/usr/bin/env python3
'''
Example script to read the sensor data.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial
- ros2 run ariac_tutorials read_break_beam_sensor.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    '''
    main function for the read_sensor script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
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

#!/usr/bin/env python3
'''
Example script to read the data from an advanced logical camera.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=ariac_tutorials trial_name:=tutorial
- ros2 run ariac_tutorials read_advanced_camera.py
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
            # interface.get_logger().info(
                # f'Number of parts detected: {len(interface.camera_images)}', throttle_duration_sec=2.0)
            
            if len(interface.camera_images) > 0:
                for image in interface.camera_images:
                    interface.get_logger().info(interface.parse_advanced_camera_image(image), throttle_duration_sec=2.0)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

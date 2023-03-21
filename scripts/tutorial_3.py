#!/usr/bin/env python3
'''
To test this script, run the following commands in separate terminals:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_tutorials
- ros2 run ariac_tutorials tutorial_3.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface


def main(args=None):
    rclpy.init(args=args)
    interface = CompetitionInterface()
    interface.start_competition()

    while rclpy.ok():
        try:
            rclpy.spin_once(interface)
            image = interface.camera_image
            if image is not None:
                interface.get_logger().info(interface.parse_advanced_camera_image(image), throttle_duration_sec=5.0)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
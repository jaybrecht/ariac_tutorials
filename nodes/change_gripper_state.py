#!/usr/bin/env python3
'''
Example script to change the gripper state of the floor robot.

To test this script, run the following command:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials change_gripper_state.py
'''

import rclpy
from ariac_tutorials.competition_interface import CompetitionInterface

def main(args=None):
    '''
    main function for the change_gripper_state script.

    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)

    interface = CompetitionInterface()

    interface.start_competition()

    while rclpy.ok():
        try:
            interface.set_floor_robot_gripper_state(True)
            interface.wait(3)
            interface.set_floor_robot_gripper_state(False)
            interface.wait(3)
        except KeyboardInterrupt:
            break

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
'''
Example script to move the robot using ROS2 Actions
To test this script, run the following command:
- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial
- ros2 run ariac_tutorials tutorial_8.py
'''

import rclpy
from rclpy.executors import MultiThreadedExecutor
from ariac_tutorials.competition_interface import CompetitionInterface
from ariac_tutorials.robot_commander import RobotCommander, GoalRejectedException


def main(args=None):
    '''
    main function for the move_robot_with_action_client script.
    Args:
        args (Any, optional): ROS arguments. Defaults to None.
    '''
    rclpy.init(args=args)

    interface = CompetitionInterface()
    commander = RobotCommander()

    executor = MultiThreadedExecutor()

    executor.add_node(interface)
    executor.add_node(commander)

    interface.start_competition()
    interface.wait(3)
    
    # Move robots to home position
    commander.get_logger().info("Moving the robots to the home position")
    commander.move_floor_robot(RobotCommander.floor_robot_home_, 2)
    commander.move_ceiling_robot(RobotCommander.ceiling_robot_home_, 4)
    commander.get_logger().info("Waiting for both robots to finish moving..")
    
    try:
        while not commander.finished_executing():
            executor.spin_once()
    except GoalRejectedException as e:
        print(e)

    interface.wait(3)

    # Move robots to second position
    commander.get_logger().info("Moving the robots to the second position")
    commander.move_floor_robot(RobotCommander.floor_robot_position2_, 4)
    commander.move_ceiling_robot(RobotCommander.ceiling_robot_position2_, 2)
    commander.get_logger().info("Waiting for both robots to finish moving..")
    try:
        while not commander.finished_executing():
            executor.spin_once()
    except GoalRejectedException as e:
        print(e)



if __name__ == '__main__':
    main()
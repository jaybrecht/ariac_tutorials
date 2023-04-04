#!/usr/bin/env python3

from math import pi

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class GoalRejectedException(Exception):
    pass

class Status():
    REQUESTED = 1
    ACCEPTED = 2
    REJECTED = 3
    FINISHED = 4

class RobotCommander(Node):
    floor_robot_joint_names = ['floor_shoulder_pan_joint',
                               'floor_shoulder_lift_joint',
                               'floor_elbow_joint',
                               'floor_wrist_1_joint',
                               'floor_wrist_2_joint',
                               'floor_wrist_3_joint',]

    floor_robot_home_ = [0.0, -pi/2, pi/2, -pi/2, -pi/2, 0.0]
    floor_robot_position2_ = [0.0, -pi/2, 0.0, -pi/2, -pi/2, 0.0]

    ceiling_robot_joint_names = ['ceiling_shoulder_pan_joint',
                                 'ceiling_shoulder_lift_joint',
                                 'ceiling_elbow_joint',
                                 'ceiling_wrist_1_joint',
                                 'ceiling_wrist_2_joint',
                                 'ceiling_wrist_3_joint',]

    ceiling_robot_home_ = [0.0, -pi/2, pi/2, pi, -pi/2, 0.0]
    ceiling_robot_position2_ = [0.0, -pi/2, 0.0, pi, -pi/2, 0.0]

    def __init__(self):
        super().__init__('robot_control')

        sim_time = Parameter(
            "use_sim_time",
            Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self._floor_robot_action_client = ActionClient(
            self, FollowJointTrajectory, '/floor_robot_controller/follow_joint_trajectory')

        self._ceiling_robot_action_client = ActionClient(
            self, FollowJointTrajectory, '/ceiling_robot_controller/follow_joint_trajectory')

        self.floor_robot_goal_status = 0
        self.ceiling_robot_goal_status = 0

    def move_floor_robot(self, joint_positions, move_time):
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=move_time).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.floor_robot_joint_names
        goal_msg.trajectory.points.append(point)

        self._floor_robot_action_client.wait_for_server()

        self.floor_robot_goal_status = Status.REQUESTED
        self._floor_robot_send_goal_future = self._floor_robot_action_client.send_goal_async(
            goal_msg)

        self._floor_robot_send_goal_future.add_done_callback(
            self.floor_robot_goal_response_callback)

    def floor_robot_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.floor_robot_goal_status = Status.REJECTED
            self.get_logger().info('Goal rejected')
            return

        self.floor_robot_goal_status = Status.ACCEPTED
        self.get_logger().info('Goal accepted')

        self._floor_robot_get_result_future = goal_handle.get_result_async()
        self._floor_robot_get_result_future.add_done_callback(
            self.floor_robot_get_result_callback)
        
    def floor_robot_get_result_callback(self, future):
        result = future.result().result
        result: FollowJointTrajectory.Result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Move succeeded")
        else:
            self.get_logger().error(result.error_string)

        self.floor_robot_goal_status = Status.FINISHED

    def move_ceiling_robot(self, joint_positions, move_time):
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=move_time).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.ceiling_robot_joint_names
        goal_msg.trajectory.points.append(point)

        self._ceiling_robot_action_client.wait_for_server()

        self.ceiling_robot_goal_status = Status.REQUESTED
        self._ceiling_robot_send_goal_future = self._ceiling_robot_action_client.send_goal_async(
            goal_msg)

        self._ceiling_robot_send_goal_future.add_done_callback(
            self.ceiling_robot_goal_response_callback)
    
    def ceiling_robot_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.ceiling_robot_goal_status = Status.REJECTED
            self.get_logger().info('Goal rejected')
            return

        self.ceiling_robot_goal_status = Status.ACCEPTED
        self.get_logger().info('Goal accepted')

        self._ceiling_robot_get_result_future = goal_handle.get_result_async()
        self._ceiling_robot_get_result_future.add_done_callback(
            self.ceiling_robot_get_result_callback)
    
    def ceiling_robot_get_result_callback(self, future):
        result = future.result().result
        result: FollowJointTrajectory.Result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Move succeeded")
        else:
            self.get_logger().error(result.error_string)

        self.ceiling_robot_goal_status = Status.FINISHED

    def finished_executing(self):
        if self.floor_robot_goal_status == Status.REJECTED:
            raise GoalRejectedException("floor robot rejected the goal")
        
        if self.ceiling_robot_goal_status == Status.REJECTED:
            raise GoalRejectedException("ceiling robot rejected the goal")

        if (self.floor_robot_goal_status == Status.FINISHED and
                self.ceiling_robot_goal_status == Status.FINISHED):
            return True
        return False
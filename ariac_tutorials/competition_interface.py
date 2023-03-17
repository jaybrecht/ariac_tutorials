#!/usr/bin/env python3
'''
Module used in tutorials.
'''

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from ariac_tutorials.utils import multiply_pose

from ariac_msgs.msg import (
    CompetitionState,
    BreakBeamStatus,
    Part,
    PartPose,
    AdvancedLogicalCameraImage,
)

from geometry_msgs.msg import Pose

from std_srvs.srv import Trigger


class CompetitionInterface(Node):
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    _competition_states = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''

    _part_colors = {
        Part.RED: 'red',
        Part.BLUE: 'blue',
        Part.GREEN: 'green',
        Part.ORANGE: 'orange',
        Part.PURPLE: 'purple',
    }
    '''Dictionary for converting PartColor constants to strings'''

    _part_colors_emoji = {
        Part.RED: '🟥',
        Part.BLUE: '🟦',
        Part.GREEN: '🟩',
        Part.ORANGE: '🟧',
        Part.PURPLE: '🟪',
    }
    '''Dictionary for displaying an emoji for the part color'''

    _part_types = {
        Part.BATTERY: 'battery',
        Part.PUMP: 'pump',
        Part.REGULATOR: 'regulator',
        Part.SENSOR: 'sensor',
    }
    '''Dictionary for converting PartType constants to strings'''

    def __init__(self):
        super().__init__('competition_interface')

        # Sets the node to use simulation time
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time])

        # Flag for parsing incoming competition state
        self.competition_state = None

        # Flag for detecting objects on conveyor belt
        self.conveyor_object_detected = False

        # Counter of parts on the conveyor
        self.conveyor_part_count = 0

        # Flag for initial receipt of camera image
        self.received_data_from_camera = False

        # Camera Image
        self.camera_image = AdvancedLogicalCameraImage()

        # Subscriber to the competition state topic
        self.subscription = self.create_subscription(
            CompetitionState,
            '/ariac/competition_state',
            self.competition_state_cb,
            10)

        # Subscriber to the break beam status topic
        self.break_beam_sub = self.create_subscription(
            BreakBeamStatus,
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos_profile_sensor_data)

        # Subscriber to the logical camera topic
        self.advanced_camera0_sub = self.create_subscription(
            AdvancedLogicalCameraImage,
            '/ariac/sensors/advanced_camera_0/image',
            self.advanced_camera0_cb,
            qos_profile_sensor_data)

        # Service client for starting the competition
        self.competition_starter = self.create_client(
            Trigger, '/ariac/start_competition')

    def competition_state_cb(self, msg: CompetitionState):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {CompetitionInterface._competition_states[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def start_competition(self):
        '''Function to start the competition.
        '''

        if self.competition_state == CompetitionState.STARTED:
            self.get_logger().warn('Competition is already started.')
            return

        # Wait for competition to be ready
        while self.competition_state != CompetitionState.READY:
            self.get_logger().info('Waiting for competition to be ready.', once=True)
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Check if service is available
        if not self.competition_starter.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
            return

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self.competition_starter.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().error('Unable to start competition.')

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        '''Callback for the topic /ariac/sensors/breakbeam_0/status
        Arguments:
            msg -- BreakBeamStatus message
        '''

        if not self.conveyor_object_detected and msg.object_detected:
            self.conveyor_part_count += 1

        # Store the last reading from the sensor
        self.conveyor_object_detected = msg.object_detected

    def advanced_camera0_cb(self, msg: AdvancedLogicalCameraImage):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image
        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''

        self.received_data_from_camera = True
        self.camera_image = msg

    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage):
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.

        Args:
            image (AdvancedLogicalCameraImage): Object of type AdvancedLogicalCameraImage
        '''
        output = '\n\n==========================\n'

        sensor_pose: Pose = image.sensor_pose

        part_pose: PartPose
        for part_pose in image.part_poses:
            part_color = CompetitionInterface._part_colors[
                part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[
                part_pose.part.color]
            part_type = CompetitionInterface._part_types[
                part_pose.part.type].capitalize()
            output += f'Part: {part_color_emoji} {part_color} {part_type}\n'
            output += '==========================\n'
            output += 'Camera Frame\n'
            output += '==========================\n'
            position = f'x: {part_pose.pose.position.x}\n\t\ty: {part_pose.pose.position.y}\n\t\tz: {part_pose.pose.position.z}'
            orientation = f'x: {part_pose.pose.orientation.x}\n\t\ty: {part_pose.pose.orientation.y}\n\t\tz: {part_pose.pose.orientation.z}\n\t\tw: {part_pose.pose.orientation.w}'

            output += '\tPosition:\n'
            output += f'\t\t{position}\n'
            output += '\tOrientation:\n'
            output += f'\t\t{orientation}\n'
            output += '==========================\n'
            output += 'World Frame\n'
            output += '==========================\n'
            part_world_pose = multiply_pose(sensor_pose, part_pose.pose)
            position = f'x: {part_world_pose.position.x}\n\t\ty: {part_world_pose.position.y}\n\t\tz: {part_world_pose.position.z}'
            orientation = f'x: {part_world_pose.orientation.x}\n\t\ty: {part_world_pose.orientation.y}\n\t\tz: {part_world_pose.orientation.z}\n\t\tw: {part_world_pose.orientation.w}'

            output += '\tPosition:\n'
            output += f'\t\t{position}\n'
            output += '\tOrientation:\n'
            output += f'\t\t{orientation}\n'
            output += '==========================\n'

        return output

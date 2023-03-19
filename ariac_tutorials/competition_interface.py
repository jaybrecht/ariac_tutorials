#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
)

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
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        # Service client for starting the competition
        self._start_competition_client = self.create_client(Trigger, '/ariac/start_competition')

        # Subscriber to the competition state topic
        self._competition_state_sub = self.create_subscription(
            CompetitionStateMsg,
            '/ariac/competition_state',
            self.competition_state_cb,
            10)
        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        # Subscriber to the break beam status topic
        self._break_beam0_sub = self.create_subscription(
            BreakBeamStatusMsg,
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos_profile_sensor_data)
        # Store the number of parts that crossed the beam
        self._part_count = 0
        # Store whether the beam is broken
        self._object_detected = False

    @property
    def part_count(self):
        '''Number of parts that crossed the beam.'''
        return self._part_count

    def breakbeam0_cb(self, msg: BreakBeamStatusMsg):
        '''Callback for the topic /ariac/sensors/breakbeam_0/status

        Arguments:
            msg -- BreakBeamStatusMsg message
        '''
        if not self._object_detected and msg.object_detected:
            self._part_count += 1

        self._object_detected = msg.object_detected

    def competition_state_cb(self, msg: CompetitionStateMsg):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self._competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {CompetitionInterface._competition_states[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self._competition_state = msg.competition_state

    def start_competition(self):
        '''Function to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        if self._competition_state == CompetitionStateMsg.STARTED:
            return
        # Wait for competition to be ready
        while self._competition_state != CompetitionStateMsg.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self._start_competition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')

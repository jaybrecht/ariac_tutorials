import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose
from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
    BreakBeamStatus as BreakBeamStatusMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    Part as PartMsg,
    PartPose as PartPoseMsg,
    Order as OrderMsg,
    AssemblyPart as AssemblyPartMsg,
    AssemblyTask as AssemblyTaskMsg,
    AGVStatus as AGVStatusMsg,
)

from std_srvs.srv import Trigger

from ariac_tutorials.utils import (
    multiply_pose,
    AdvancedLogicalCameraImage
)


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
    
    _part_colors = {
        PartMsg.RED: 'red',
        PartMsg.BLUE: 'blue',
        PartMsg.GREEN: 'green',
        PartMsg.ORANGE: 'orange',
        PartMsg.PURPLE: 'purple',
    }
    '''Dictionary for converting Part color constants to strings'''

    _part_colors_emoji = {
        PartMsg.RED: '🟥',
        PartMsg.BLUE: '🟦',
        PartMsg.GREEN: '🟩',
        PartMsg.ORANGE: '🟧',
        PartMsg.PURPLE: '🟪',
    }
    '''Dictionary for converting Part color constants to emojis'''

    _part_types = {
        PartMsg.BATTERY: 'battery',
        PartMsg.PUMP: 'pump',
        PartMsg.REGULATOR: 'regulator',
        PartMsg.SENSOR: 'sensor',
    }
    '''Dictionary for converting Part type constants to strings'''
    

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
            self._competition_state_cb,
            10)
        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        # Subscriber to the break beam status topic
        self._break_beam0_sub = self.create_subscription(
            BreakBeamStatusMsg,
            '/ariac/sensors/breakbeam_0/status',
            self._breakbeam0_cb,
            qos_profile_sensor_data)
        # Store the number of parts that crossed the beam
        self._conveyor_part_count = 0
        # Store whether the beam is broken
        self._object_detected = False
        
        # Subscriber to the logical camera topic
        self._advanced_camera0_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/advanced_camera_0/image',
            self._advanced_camera0_cb,
            qos_profile_sensor_data)
        # Store each camera image as an AdvancedLogicalCameraImage object
        self._camera_image: AdvancedLogicalCameraImage = None
        

    @property
    def camera_image(self):
        return self._camera_image

    @property
    def conveyor_part_count(self):
        return self._conveyor_part_count
   
    def _advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image

        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        self._camera_image = AdvancedLogicalCameraImage(msg.part_poses,
                                                        msg.tray_poses,
                                                        msg.sensor_pose)

    def _breakbeam0_cb(self, msg: BreakBeamStatusMsg):
        '''Callback for the topic /ariac/sensors/breakbeam_0/status

        Arguments:
            msg -- BreakBeamStatusMsg message
        '''
        if not self._object_detected and msg.object_detected:
            self._conveyor_part_count += 1

        self._object_detected = msg.object_detected

    def _competition_state_cb(self, msg: CompetitionStateMsg):
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
            
    

    def parse_advanced_camera_image(self):
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
        '''
        output = '\n\n==========================\n'

        sensor_pose: Pose = self._camera_image._sensor_pose

        part_pose: PartPoseMsg

        counter = 1
        for part_pose in self._camera_image._part_poses:
            part_color = CompetitionInterface._part_colors[part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface._part_colors_emoji[part_pose.part.color]
            part_type = CompetitionInterface._part_types[part_pose.part.type].capitalize()
            output += f'Part {counter}: {part_color_emoji} {part_color} {part_type}\n'
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

            counter += 1

        return output
    
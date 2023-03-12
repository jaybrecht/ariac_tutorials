#!/usr/bin/env python3
'''
Module used in tutorials.
'''


import rclpy
import PyKDL
import tf2_kdl
from dataclasses import dataclass
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose

from ariac_msgs.msg import (
    CompetitionState,
    BreakBeamStatus,
    Order as OrderMsg,
    Part,
    KittingPart as KittingPartMsg,
    AssemblyPart as AssemblyPartMsg,
    AGVStatus,
    VacuumGripperState,
    AssemblyTask as AssemblyTaskMsg,
    KittingTask as KittingTaskMsg,
    CombinedTask as CombinedTaskMsg,
    AdvancedLogicalCameraImage as AdvancedLogicalCameraImageMsg,
    PartPose as PartPoseMsg,
)

from ariac_msgs.srv import (
    VacuumGripperControl,
    MoveAGV
)

from std_srvs.srv import Trigger


@dataclass
class KittingPart:
    '''
    Class to store information about a KittingPartMsg.
    '''
    quadrant: int
    part_type: int
    part_color: int
        

class KittingTask:
    '''
    Class to store information about a KittingTaskMsg.
    '''
    def __init__(self, kitting_task_msg: KittingTaskMsg) -> None:
        self.agv_number = kitting_task_msg.agv_number
        self.tray_id = kitting_task_msg.tray_id
        self.destination = kitting_task_msg.destination
        # list of KittingPart objects
        self.parts = list(map(lambda x: KittingPart(x.quadrant, x.part.type, x.part.color), kitting_task_msg.parts))
        # self.parts = list(map(lambda x: KittingPart(x), kitting_task_msg.parts))

class AssemblyPart:
    '''
    Class to store information about a AssemblyPartMsg.
    '''
    def __init__(self, assembly_part: AssemblyPartMsg) -> None:
        self.part_type = assembly_part.part.type
        self.part_color = assembly_part.part.color
        self.assembled_pose = assembly_part.assembled_pose
        self.install_direction = assembly_part.install_direction

class AssemblyTask:
    '''
    Class to store information about a AssemblyTaskMsg.
    '''
    def __init__(self, assembly_task: AssemblyTaskMsg) -> None:
        self.agv_numbers = assembly_task.agv_numbers
        self.station = assembly_task.station
        # list of AssemblyPart objects
        self.parts = list(map(lambda x: AssemblyPart(x), assembly_task.parts))

class CombinedTask:
    '''
    Class to store information about a CombinedTaskMsg.
    '''

    def __init__(self, combined_task: CombinedTaskMsg) -> None:
        self.station = combined_task.station
        # list of AssemblyPart objects
        self.parts = list(map(lambda x: AssemblyPart(x), combined_task.parts))
        
class Order:
    ''' Order class for storing order information from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority

        if self.order_type == OrderMsg.KITTING:
            self.order_task = KittingTask(msg.kitting_task)
        elif self.order_type == OrderMsg.ASSEMBLY:
            self.order_task = AssemblyTask(msg.assembly_task)
        elif self.order_type == OrderMsg.COMBINED:
            self.order_task = CombinedTask(msg.combined_task)
        else:
            self.order_task = None
            
class AdvancedLogicalCameraImage:
    '''
    Class to store information about a AdvancedLogicalCameraImage.
    '''

    def __init__(self, msg: AdvancedLogicalCameraImageMsg) -> None:
        self.part_poses = msg.part_poses
        self.tray_poses = msg.tray_poses
        self.sensor_pose = msg.sensor_pose

class CompetitionInterface(Node):
    '''
    Class for a competition interface node.

    Args:
        Node (rclpy.node.Node): Parent class for ROS nodes

    Raises:
        KeyboardInterrupt: Exception raised when the user uses Ctrl+C to kill a process
    '''

    part_colors_ = {
        Part.RED: 'red',
        Part.BLUE: 'blue',
        Part.GREEN: 'green',
        Part.ORANGE: 'orange',
        Part.PURPLE: 'purple',
    }
    
    part_colors_emoji_ = {
        Part.RED: '🟥',
        Part.BLUE: '🟦',
        Part.GREEN: '🟩',
        Part.ORANGE: '🟧',
        Part.PURPLE: '🟪',
    }

    '''Dictionary for converting PartColor constants to strings'''

    part_types_ = {
        Part.BATTERY: 'battery',
        Part.PUMP: 'pump',
        Part.REGULATOR: 'regulator',
        Part.SENSOR: 'sensor',
    }
    '''Dictionary for converting PartType constants to strings'''


    competition_states_ = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }
    '''Dictionary for converting CompetitionState constants to strings'''


    destinations_ = {
        AGVStatus.KITTING: 'kitting station',
        AGVStatus.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatus.ASSEMBLY_BACK: 'back assembly station',
        AGVStatus.WAREHOUSE: 'warehouse',
    }
    '''Dictionary for converting AGVDestination constants to strings'''

    stations_ = {
        AssemblyTaskMsg.AS1: "assembly station 1",
        AssemblyTaskMsg.AS2: "assembly station 2",
        AssemblyTaskMsg.AS3: "assembly station 3",
        AssemblyTaskMsg.AS4: "assembly station 4",
    }
    '''Dictionary for converting AssemblyTaskMsg constants to strings'''

    gripper_states_ = {
        True: 'enabled',
        False: 'disabled'
    }
    '''Dictionary for converting VacuumGripperState constants to strings'''

    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])
        
        # Flag for parsing incoming orders
        self.parse_incoming_order = False
        # Flag for parsing incoming competition state
        self.competition_state = None

        # Subscriber to the competition state topic
        self.subscription = self.create_subscription(
            CompetitionState,
            '/ariac/competition_state',
            self.competition_state_cb,
            10)
        # Subscriber to the floor gripper state topic
        self.floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            '/ariac/floor_robot_gripper_state',
            self.floor_robot_gripper_state_cb,
            qos_profile_sensor_data)
        # Subscriber to the break beam status topic
        self.break_beam_sub = self.create_subscription(
            BreakBeamStatus,
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos_profile_sensor_data)
        # Subscriber to the logical camera topic
        self.advanced_camera0_sub = self.create_subscription(
            AdvancedLogicalCameraImageMsg,
            '/ariac/sensors/advanced_camera_0/image',
            self.advanced_camera0_cb,
            qos_profile_sensor_data)
        # Subscriber to the order topic
        self.orders_sub = self.create_subscription(
            OrderMsg,
            '/ariac/orders',
            self.orders_cb,
            10
        )

        # Service client for starting the competition
        self.starter = self.create_client(Trigger, '/ariac/start_competition')
        # Service client for turning on/off the vacuum gripper on the floor robot
        self.floor_gripper_enable = self.create_client(
            VacuumGripperControl,
            "/ariac/floor_robot_enable_gripper")
        # Service client for moving the floor robot to the home position
        self.move_floor_robot_home = self.create_client(
            Trigger, '/competitor/move_floor_robot_home')
        # Service client for moving the ceiling robot to the home position
        self.move_ceiling_robot_home = self.create_client(
            Trigger, '/competitor/move_ceiling_robot_home')

        self.part_count = 0
        self.object_detected = False
        # List of orders
        self.orders = []
        self.camera_images = []
        # Attribute to store the current state of the floor robot gripper
        self.floor_robot_gripper_state = VacuumGripperState()

    def competition_state_cb(self, msg: CompetitionState):
        '''Callback for the topic /ariac/competition_state

        Arguments:
            msg -- CompetitionState message
        '''
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: \
                {CompetitionInterface.competition_states_[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def start_competition(self):
        '''Function to start the competition.
        '''
        self.get_logger().info('Waiting for competition to be ready')

        if self.competition_state == CompetitionState.STARTED:
            return
        # Wait for competition to be ready
        while self.competition_state != CompetitionState.READY:
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                return

        self.get_logger().info('Competition is ready. Starting...')

        # Call ROS service to start competition
        while not self.starter.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /ariac/start_competition to be available...')

        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self.starter.call_async(request)

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().info('Unable to start competition')
            
    def advanced_camera0_cb(self, msg: AdvancedLogicalCameraImageMsg):
        '''Callback for the topic /ariac/sensors/advanced_camera_0/image

        Arguments:
            msg -- AdvancedLogicalCameraImage message
        '''
        image = AdvancedLogicalCameraImage(msg)
        self.camera_images.clear()
        self.camera_images.append(image)

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        '''Callback for the topic /ariac/sensors/breakbeam_0/status

        Arguments:
            msg -- BreakBeamStatus message
        '''
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected

    def orders_cb(self, msg: Order):
        '''Callback for the topic /ariac/orders

        Arguments:
            msg -- Order message
        '''
        order = Order(msg)
        self.orders.append(order)
        if self.parse_incoming_order:
            self.get_logger().info(self.parse_order(order))
            
    def multiply_pose(self, pose1: Pose, pose2: Pose):
        '''
        Use KDL to multiply two poses together.

        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame

        Returns:
            Pose: Pose of the resulting frame
        '''
        
        frame1 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w), PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))
        frame2 = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w), PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))
        
        frame3: PyKDL.Frame = frame1 * frame2
        
        # return the resulting pose from frame3
        tf2 = Pose()
        tf2.position.x = frame3.p.x()
        tf2.position.y = frame3.p.y()
        tf2.position.z = frame3.p.z()
        tf2.orientation.x = frame3.M.GetQuaternion()[0]
        tf2.orientation.y = frame3.M.GetQuaternion()[1]
        tf2.orientation.z = frame3.M.GetQuaternion()[2]
        tf2.orientation.w = frame3.M.GetQuaternion()[3]
        
        return tf2
        
    def parse_advanced_camera_image(self, image: AdvancedLogicalCameraImage):
        '''
        Parse an AdvancedLogicalCameraImage message and return a string representation.
    

        Args:
            image (AdvancedLogicalCameraImage): Object of type AdvancedLogicalCameraImage
        '''
        output = '\n\n==========================\n'
        
        sensor_pose: Pose = image.sensor_pose
        
        part_pose: PartPoseMsg
        for part_pose in image.part_poses:
            part_color = CompetitionInterface.part_colors_[part_pose.part.color].capitalize()
            part_color_emoji = CompetitionInterface.part_colors_emoji_[part_pose.part.color]
            part_type = CompetitionInterface.part_types_[part_pose.part.type].capitalize()
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
            part_world_pose = self.multiply_pose(sensor_pose, part_pose.pose)
            position = f'x: {part_world_pose.position.x}\n\t\ty: {part_world_pose.position.y}\n\t\tz: {part_world_pose.position.z}'
            orientation = f'x: {part_world_pose.orientation.x}\n\t\ty: {part_world_pose.orientation.y}\n\t\tz: {part_world_pose.orientation.z}\n\t\tw: {part_world_pose.orientation.w}'

            output += '\tPosition:\n'
            output += f'\t\t{position}\n'
            output += '\tOrientation:\n'
            output += f'\t\t{orientation}\n'
            output += '==========================\n'
        
        return output

    def parse_order(self, order: Order):
        '''Parse an order message and return a string representation.

        Args:
            order -- Order message

        Returns:
            String representation of the order message
        '''
        output = '\n\n==========================\n'
        output += f'Received Order: {order.order_id}\n'
        output += f'Priority: {order.order_priority}\n'

        if order.order_type == OrderMsg.KITTING:
            kitting_task: KittingTask = order.order_task
            output += 'Type: Kitting\n'
            output += '==========================\n'
            output += f'AGV: {kitting_task.agv_number}\n'
            output += f'Destination: {self.destinations_[kitting_task.destination]}\n'
            output += f'Tray ID: {kitting_task.tray_id}\n'
            output += 'Products:\n'
            output += '==========================\n'

            quadrants = {1: "Quadrant 1: -",
                         2: "Quadrant 2: -",
                         3: "Quadrant 3: -",
                         4: "Quadrant 4: -"}

            for i in range(1, 5):
                product: KittingPartMsg
                for product in kitting_task.parts:
                    if i == product.quadrant:
                        part_color = CompetitionInterface.part_colors_[product.part_color].capitalize()
                        part_color_emoji = CompetitionInterface.part_colors_emoji_[product.part_color]
                        part_type = CompetitionInterface.part_types_[product.part_type].capitalize()
                        quadrants[i] = f'Quadrant {i}: {part_color_emoji} {part_color} {part_type}'
            output += f'\t{quadrants[1]}\n'
            output += f'\t{quadrants[2]}\n'
            output += f'\t{quadrants[3]}\n'
            output += f'\t{quadrants[4]}\n'

        elif order.order_type == OrderMsg.ASSEMBLY:
            assembly_task: AssemblyTask = order.order_task
            output += 'Type: Assembly\n'
            output += '==========================\n'
            if len(assembly_task.agv_numbers) == 1:
                output += f'AGV: {assembly_task.agv_number[0]}\n'
            elif len(assembly_task.agv_numbers) == 2:
                output += f'AGV(s): [{assembly_task.agv_numbers[0]}, {assembly_task.agv_numbers[1]}]\n'
            output += f'Assembly station: {self.destinations_[assembly_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in assembly_task.parts:
                part_color = CompetitionInterface.part_colors_[product.part_color].capitalize()
                part_color_emoji = CompetitionInterface.part_colors_emoji_[product.part_color]
                part_type = CompetitionInterface.part_types_[product.part_type].capitalize()
                assembled_pose_position = product.assembled_pose.pose.position
                assembled_pose_orientation = product.assembled_pose.pose.orientation
                install_direction = product.install_direction
                position = f'x: {assembled_pose_position.x}\n\t\ty: {assembled_pose_position.y}\n\t\tz: {assembled_pose_position.z}'
                orientation = f'x: {assembled_pose_orientation.x}\n\t\ty: {assembled_pose_orientation.y}\n\t\tz: {assembled_pose_orientation.z}\n\t\tw: {assembled_pose_orientation.w}'
                output += f'\tPart: {part_color_emoji} {part_color} {part_type}\n'
                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += f'\tInstall direction: [{install_direction.x}, {install_direction.y}, {install_direction.z}]\n\n'
        
        elif order.order_type == OrderMsg.COMBINED:
            combined_task: CombinedTask = order.order_task
            output += 'Type: Combined\n'
            output += '==========================\n'
            output += f'Assembly station: {self.destinations_[combined_task.station].title()}\n'
            output += 'Products:\n'
            output += '==========================\n'

            product: AssemblyPartMsg
            for product in combined_task.parts:
                part_color = CompetitionInterface.part_colors_[product.part_color].capitalize()
                part_color_emoji = CompetitionInterface.part_colors_emoji_[product.part_color]
                part_type = CompetitionInterface.part_types_[product.part_type].capitalize()
                assembled_pose_position = product.assembled_pose.pose.position
                assembled_pose_orientation = product.assembled_pose.pose.orientation
                install_direction = product.install_direction
                position = f'x: {assembled_pose_position.x}\n\t\ty: {assembled_pose_position.y}\n\t\tz: {assembled_pose_position.z}'
                orientation = f'x: {assembled_pose_orientation.x}\n\t\ty: {assembled_pose_orientation.y}\n\t\tz: {assembled_pose_orientation.z}\n\t\tw: {assembled_pose_orientation.w}'
                output += f'\tPart: {part_color_emoji} {part_color} {part_type}\n'
                output += '\tPosition:\n'
                output += f'\t\t{position}\n'
                output += '\tOrientation:\n'
                output += f'\t\t{orientation}\n'
                output += f'\tInstall direction: [{install_direction.x}, {install_direction.y}, {install_direction.z}]\n\n'
        else:
            output += 'Type: Unknown\n'
        return output

    def floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        '''Callback for the topic /ariac/floor_robot_gripper_state

        Arguments:
            msg -- VacuumGripperState message
        '''
        self.floor_robot_gripper_state = msg

    def set_floor_robot_gripper_state(self, state):
        '''Set the gripper state of the floor robot.

        Arguments:
            state -- True to enable, False to disable

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        if self.floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f'Gripper is already {self.gripper_states_[state]}')
            return

        request = VacuumGripperControl.Request()
        request.enable = state

        future = self.floor_gripper_enable.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().info(f'Changed gripper state to {self.gripper_states_[state]}')
        else:
            self.get_logger().warn('Unable to change gripper state')

    def wait(self, duration):
        '''Wait for a specified duration.

        Arguments:
            duration -- Duration to wait in seconds

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        start = self.get_clock().now()

        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt as kb_error:
                raise KeyboardInterrupt from kb_error

    def lock_agv_tray(self, num):
        '''Function to lock the tray of an AGV.

        Arguments:
            num -- AGV number

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        tray_locker = self.create_client(
            Trigger,
            f'/ariac/agv{num}_lock_tray'
        )

        request = Trigger.Request()

        future = tray_locker.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().info(f'Locked AGV{num}\'s tray')
        else:
            self.get_logger().warn('Unable to lock tray')

    def move_agv_to_station(self, num, station):
        '''Function to move an AGV to a station.

        Arguments:
            num -- AGV number

            station -- Station to move to

        Raises:
            KeyboardInterrupt: Exception raised when the user presses Ctrl+C
        '''
        mover = self.create_client(
            MoveAGV,
            f'/ariac/move_agv{num}')

        request = MoveAGV.Request()

        if station in [AssemblyTaskMsg.AS1, AssemblyTaskMsg.AS3]:
            request.location = MoveAGV.Request.ASSEMBLY_FRONT
        else:
            request.location = MoveAGV.Request.ASSEMBLY_BACK

        future = mover.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt as kb_error:
            raise KeyboardInterrupt from kb_error

        if future.result().success:
            self.get_logger().info(f'Moved AGV{num} to {self.stations_[station]}')
        else:
            self.get_logger().warn(future.result().message)

    def move_robot_home(self, robot_name):
        '''Move one of the robots to its home position.

        Arguments:
            robot_name -- Name of the robot to move home
        '''
        request = Trigger.Request()

        if robot_name == 'floor_robot':
            if not self.move_floor_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return

            future = self.move_floor_robot_home.call_async(request)

        elif robot_name == 'ceiling_robot':
            if not self.move_ceiling_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return
            future = self.move_ceiling_robot_home.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return

        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)

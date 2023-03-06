#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration
from rclpy.parameter import Parameter

from ariac_msgs.msg import (
    CompetitionState,
    BreakBeamStatus,
    Order,
    Part,
    KittingPart,
    AGVStatus,
    VacuumGripperState,
    AssemblyTask,
    )

from std_srvs.srv import Trigger
from ariac_msgs.srv import (
    VacuumGripperControl,
    MoveAGV
    )

class CompetitionInterface(Node):
    competition_states_ = {
        CompetitionState.IDLE: 'idle',
        CompetitionState.READY: 'ready',
        CompetitionState.STARTED: 'started',
        CompetitionState.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionState.ENDED: 'ended',
    }

    # Dictionary to convert part_color constants to strings
    colors_ = {
        Part.RED: 'red',
        Part.BLUE: 'blue',
        Part.GREEN: 'green',
        Part.ORANGE: 'orange',
        Part.PURPLE: 'purple',
    }

    # Dictionary to convert part_type constants to strings
    part_types_ = {
        Part.BATTERY: 'battery',
        Part.PUMP: 'pump',
        Part.REGULATOR: 'regulator',
        Part.SENSOR: 'sensor',
    }

    # Dictionary to convert agv destination constants to strings
    destinations_ = {
        AGVStatus.KITTING: 'kitting station',
        AGVStatus.ASSEMBLY_FRONT: 'front assembly station',
        AGVStatus.ASSEMBLY_BACK: 'back assembly station',
        AGVStatus.WAREHOUSE: 'warehouse',
    }
    
    stations_ = {
        AssemblyTask.AS1: "assembly station 1",
        AssemblyTask.AS2: "assembly station 2",
        AssemblyTask.AS3: "assembly station 3",
        AssemblyTask.AS4: "assembly station 4",
    }

    gripper_states_ = {
        True: 'enabled',
        False: 'disabled'
    }
    
    def __init__(self):
        super().__init__('competition_interface')

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )

        self.set_parameters([sim_time])

        self.competition_state = None

        self.subscription = self.create_subscription(
            CompetitionState, 
            '/ariac/competition_state',
            self.competition_state_cb,
            10)
        
        self.starter = self.create_client(Trigger, '/ariac/start_competition')

        self.part_count = 0
        self.object_detected = False
        self.break_beam_sub = self.create_subscription(
            BreakBeamStatus, 
            '/ariac/sensors/breakbeam_0/status',
            self.breakbeam0_cb,
            qos_profile_sensor_data)

        self.orders = []
        self.orders_sub = self.create_subscription(
            Order,
            '/ariac/orders',
            self.orders_cb,
            10
        )

        self.floor_robot_gripper_state = VacuumGripperState()

        self.floor_gripper_enable = self.create_client(
            VacuumGripperControl, 
            "/ariac/floor_robot_enable_gripper")
        
        self.floor_robot_gripper_state_sub = self.create_subscription(
            VacuumGripperState,
            '/ariac/floor_robot_gripper_state', 
            self.floor_robot_gripper_state_cb, 
            qos_profile_sensor_data)
        
        self.move_floor_robot_home = self.create_client(
            Trigger, 
            '/competitor/move_floor_robot_home')
    
        self.move_ceiling_robot_home = self.create_client(
            Trigger, 
            '/competitor/move_ceiling_robot_home')
     
    def competition_state_cb(self, msg: CompetitionState):
        # Log if competition state has changed
        if self.competition_state != msg.competition_state:
            self.get_logger().info(
                f'Competition state is: {self.competition_states_[msg.competition_state]}',
                throttle_duration_sec=1.0)
        self.competition_state = msg.competition_state

    def start_competition(self):
        self.get_logger().info('Waiting for competition to be ready')

        # Wait for competition to be ready
        while (self.competition_state != CompetitionState.READY):
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

    def breakbeam0_cb(self, msg: BreakBeamStatus):
        if not self.object_detected and msg.object_detected:
            self.part_count += 1

        self.object_detected = msg.object_detected

    def orders_cb(self, msg: Order):
        self.orders.append(msg)

        # self.get_logger().info(self.parse_order(msg))

    def parse_order(self, order: Order):
        s = '\n\n==============\n'
        s += f'Received Order\n'
        s += '==============\n\n'
        s += f'Order ID: {order.id}\n'

        if order.type == Order.KITTING: 
            s += f'  Kitting Task:\n'
            
            s += f'    AGV: {order.kitting_task.agv_number}\n'

            s += f'    Destination: {self.destinations_[order.kitting_task.destination]}\n'

            s += f'    Tray ID: {order.kitting_task.tray_id}\n'

            s += f'    Products:\n'

            for i in range(1, 5):
                empty = True
                for product in order.kitting_task.parts:
                    product: KittingPart
                    if i == product.quadrant:
                        empty = False
                        break

                if not empty:
                    s += f'      Quadrant {i}: '      
                    s += f'{self.colors_[product.part.color].capitalize()} '
                    s += f'{self.part_types_[product.part.type]}\n'
                else:
                    s += f'      Quadrant {i}: EMPTY\n' 

        
        return s
    
    def floor_robot_gripper_state_cb(self, msg: VacuumGripperState):
        self.floor_robot_gripper_state = msg

    def set_floor_robot_gripper_state(self, state): 
        if self.floor_robot_gripper_state.enabled == state:
            self.get_logger().warn(f'Gripper is already {self.gripper_states_[state]}')
            return
        
        request = VacuumGripperControl.Request()
        request.enable = state
        
        future = self.floor_gripper_enable.call_async(request)
        
        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        
        if future.result().success:
            self.get_logger().info(f'Changed gripper state to {self.gripper_states_[state]}')
        else:
            self.get_logger().warn('Unable to change gripper state')
        
    def wait(self, duration):
        start = self.get_clock().now()
        
        while self.get_clock().now() <= start + Duration(seconds=duration):
            try:
                rclpy.spin_once(self)
            except KeyboardInterrupt:
                raise KeyboardInterrupt
            
    def lock_agv_tray(self, num):
        tray_locker = self.create_client(
            Trigger,
            f'/ariac/agv{num}_lock_tray'
        )

        request = Trigger.Request()

        future = tray_locker.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        
        if future.result().success:
            self.get_logger().info(f'Locked AGV{num}\'s tray')
        else:
            self.get_logger().warn('Unable to lock tray')

    def move_agv_to_station(self, num, station):
        mover = self.create_client(
            MoveAGV, 
            f'/ariac/move_agv{num}'
        )
        
        request = MoveAGV.Request()

        if station == AssemblyTask.AS1 or station == AssemblyTask.AS3:
            request.location = MoveAGV.Request.ASSEMBLY_FRONT
        else:
            request.location = MoveAGV.Request.ASSEMBLY_BACK

        future = mover.call_async(request)

        try:
            rclpy.spin_until_future_complete(self, future)
        except KeyboardInterrupt:
            raise KeyboardInterrupt
        
        if future.result().success:
            self.get_logger().info(f'Moved AGV{num} to {self.stations_[station]}')
        else:
            self.get_logger().warn(future.result().message)
    
    def move_robot_home(self, robot_name):
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
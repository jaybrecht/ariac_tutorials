from typing import List
import PyKDL
from dataclasses import dataclass
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped, Vector3

from ariac_msgs.msg import (
    PartPose as PartPoseMsg,
    KitTrayPose as KitTrayPoseMsg,
    Part as PartMsg,
    Order as OrderMsg
)


def multiply_pose(p1: Pose, p2: Pose) -> Pose:
    '''
    Use KDL to multiply two poses together.
    Args:
        p1 (Pose): Pose of the first frame
        p2 (Pose): Pose of the second frame
    Returns:
        Pose: Pose of the resulting frame
    '''

    o1 = p1.orientation
    frame1 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o1.x, o1.y, o1.z, o1.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    o2 = p2.orientation
    frame2 = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(o2.x, o2.y, o2.z, o2.w),
        PyKDL.Vector(p1.position.x, p1.position.y, p1.position.z))

    frame3 = frame1 * frame2

    # return the resulting pose from frame3
    pose = Pose()
    pose.position.x = frame3.p.x()
    pose.position.y = frame3.p.y()
    pose.position.z = frame3.p.z()

    q = frame3.M.GetQuaternion()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose


@dataclass
class AdvancedLogicalCameraImage:
    '''
    Class to store information about a AdvancedLogicalCameraImageMsg
    '''
    _part_poses: PartPoseMsg
    _tray_poses: KitTrayPoseMsg
    _sensor_pose: Pose
    

@dataclass
class KittingPart:
    '''
    Class to store information about a KittingPartMsg.
    '''
    _quadrant: int
    _part: PartMsg

    @property
    def quadrant(self) -> int:
        '''
        Returns the quadrant of the part.

        Returns:
            int: The quadrant of the part.
        '''
        return self._quadrant

    @property
    def part(self) -> PartMsg:
        '''
        Returns the type of the part.

        Returns:
            PartMsg: The type of the part.
        '''
        return self._part


@dataclass
class KittingTask:
    '''
    Class to store information about a KittingTaskMsg.
    '''
    _agv_number: int
    _tray_id: int
    _destination: int
    _parts:  List[KittingPart]

    @property
    def agv_number(self) -> int:
        '''
        Returns the AGV number.

        Returns:
            int: The AGV number.
        '''
        return self._agv_number

    @property
    def tray_id(self) -> int:
        '''
        Returns the tray ID.

        Returns:
            int: The tray ID.
        '''
        return self._tray_id

    @property
    def destination(self) -> int:
        '''
        Returns the destination.

        Returns:
            int: The destination.
        '''
        return self._destination

    @property
    def parts(self) -> List[KittingPart]:
        '''
        Returns the list of parts.

        Returns:
            List[KittingPart]: The list of parts.
        '''
        return self._parts


@dataclass
class AssemblyPart:
    '''
    Class to store information about a AssemblyPartMsg.
    '''

    _part: PartMsg
    _assembled_pose: PoseStamped
    _install_direction: Vector3

    @property
    def part(self) -> PartMsg:
        '''
        Returns the type of the part.

        Returns:
            PartMsg: The type of the part.
        '''
        return self._part

    @property
    def assembled_pose(self) -> PoseStamped:
        '''
        Returns the assembled pose of the part.

        Returns:
            PoseStamped: The assembled pose of the part.
        '''
        return self._assembled_pose

    @property
    def install_direction(self) -> Vector3:
        '''
        Returns the install direction of the part.

        Returns:
            Vector3: The install direction of the part.
        '''
        return self._install_direction


@dataclass
class AssemblyTask:
    '''
    Class to store information about a AssemblyTaskMsg.
    '''

    _agv_numbers: List[int]
    _station: int
    _parts:  List[AssemblyPart]

    @property
    def agv_numbers(self) -> List[int]:
        '''
        Returns the list of AGV numbers.

        Returns:
            List[int]: The list of AGV numbers.
        '''
        return self._agv_numbers

    @property
    def station(self) -> int:
        '''
        Returns the station.

        Returns:
            int: The station.
        '''
        return self._station

    @property
    def parts(self) -> List[AssemblyPart]:
        '''
        Returns the list of parts.

        Returns:
            List[AssemblyPart]: The list of parts.
        '''
        return self._parts


@dataclass
class CombinedTask:
    '''
    Class to store information about a CombinedTaskMsg.
    '''

    _station: int
    _parts:  List[AssemblyPart]

    @property
    def station(self) -> int:
        '''
        Returns the station.

        Returns:
            int: The station.
        '''
        return self._station

    @property
    def parts(self) -> List[AssemblyPart]:
        '''
        Returns the list of parts.

        Returns:
            List[AssemblyPart]: The list of parts.
        '''
        return self._parts


class Order:
    ''' 
    Class to store one order message from the topic /ariac/orders.
    '''

    def __init__(self, msg: OrderMsg) -> None:
        self.order_id = msg.id
        self.order_type = msg.type
        self.order_priority = msg.priority

        if self.order_type == OrderMsg.KITTING:
            self.order_task = KittingTask(msg.kitting_task.agv_number,
                                          msg.kitting_task.tray_id,
                                          msg.kitting_task.destination,
                                          msg.kitting_task.parts)

        elif self.order_type == OrderMsg.ASSEMBLY:
            self.order_task = AssemblyTask(msg.assembly_task.agv_numbers,
                                           msg.assembly_task.station,
                                           msg.assembly_task.parts)
        elif self.order_type == OrderMsg.COMBINED:
            self.order_task = CombinedTask(msg.combined_task.station, msg.combined_task.parts)
        else:
            self.order_task = None

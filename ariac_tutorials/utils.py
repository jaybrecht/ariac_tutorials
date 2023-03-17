import PyKDL
from geometry_msgs.msg import Pose


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

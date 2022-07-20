import numpy as np
import os
from enum import Enum

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Vector3
from tf import transformations as tfs

path_root = os.path.abspath(os.path.join(os.path.abspath(os.path.dirname(__file__)), '../'))



def save_file(str, filepath):
    with  open(filepath, "w") as file:
        file.write(str)

def xyz_to_mat44(pos):
    return tfs.translation_matrix((pos.x, pos.y, pos.z))

def xyzw_to_mat44(ori):
    return tfs.quaternion_matrix((ori.x, ori.y, ori.z, ori.w))

def xyzquat_to_mat44(xyzquat):
    return np.dot(tfs.translation_matrix(xyzquat[:3]), tfs.quaternion_matrix(xyzquat[3:]))

def xyzrpy2mat44(xyzrpy):
    return np.dot(tfs.translation_matrix(xyzrpy[:3]), tfs.euler_matrix(*xyzrpy[3:]))


def gen_pose(x,y,z,rx,ry,rz,rw):
    pose=geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = rx
    pose.orientation.y = ry
    pose.orientation.z = rz
    pose.orientation.w = rw
    return pose

def mat44_to_xyzrpy(matrix):
    xyz = tfs.translation_from_matrix(matrix)
    rpy = tfs.euler_from_matrix(matrix)
    return [xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]]

def mat44_to_xyzrpy_deg(matrix):
    xyz = tfs.translation_from_matrix(matrix)
    rpy = tfs.euler_from_matrix(matrix)
    return [xyz[0],xyz[1],xyz[2],rpy[0]*(180./np.pi),rpy[1]*(180./np.pi),rpy[2]*(180./np.pi)]


def mat44_to_xyzquat(matrix):
    xyz = tfs.translation_from_matrix(matrix)
    quat = tfs.quaternion_from_matrix(matrix)
    return [xyz[0],xyz[1],xyz[2],quat[0],quat[1],quat[2],quat[3]]

def mat44_to_pose(matrix):
    return gen_pose(*mat44_to_xyzquat(matrix))


def quat2msg(quat):
    """
    :param quat: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: geometry_msgs.msg.Quaternion
    """
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]
    return q


def pose_to_mat44(pose):
    """
    :param pose: geometry_msgs.msg.Pose
    :return:
    """
    return np.dot(xyz_to_mat44(pose.position), xyzw_to_mat44(pose.orientation))


def pose_to_list(pose):
    list = [pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w]
    return list


def mat44_to_transform(mat44):
    transform = geometry_msgs.msg.Transform()
    xyz = tfs.translation_from_matrix(mat44)
    transform.translation = Vector3(*xyz)
    transform.rotation = quat2msg(tfs.quaternion_from_matrix(mat44))
    return transform

def transrot_to_mat44(translation, rotation):
    """
    :param translation: translation expressed as a tuple (x,y,z)
    :param rotation: rotation quaternion expressed as a tuple (x,y,z,w)
    :return: a :class:`numpy.matrix` 4x4 representation of the transform
    :raises: any of the exceptions that :meth:`~tf.Transformer.lookupTransform` can raise

    Converts a transformation from :class:`tf.Transformer` into a representation as a 4x4 matrix.
    """

    return np.dot(tfs.translation_matrix(translation), tfs.quaternion_matrix(rotation))


def transform_to_mat44(transform):
    t = transform.translation
    r = transform.rotation
    return transrot_to_mat44([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])


def transform_point_from_tf(mat44, point):
    xyz = tuple(np.dot(mat44, np.array([point.x, point.y, point.z, 1.0])))[:3]
    point = geometry_msgs.msg.Point(*xyz)
    return point

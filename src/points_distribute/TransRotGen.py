#!/usr/bin/env python 

import numpy as np

import rospy

from geometry_msgs.msg import Quaternion

def Rotation(axis, angle, in_degree=False):
    """A funtion to return homegeneous rotation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        angle {float} -- in degrees
        in_degree {bool} -- if True do conversion from degree to radian
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix
    """
    if in_degree:
        angle = np.deg2rad(angle)
    else:
        pass
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Rot_X(angle)
        elif axis == 'y':
            return Rot_Y(angle)
        else: # axis == 'z'
            return Rot_Z(angle)
    else:
        rospy.logerr('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Rot_X(alpha):
    """A function to return homogeneous rotation matrix along x-axis
    
    Arguments:
        alpha {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along x-axis
    """
    T_rot_X = np.mat(np.eye(4,4))

    T_rot_X[1, 1] = np.cos(alpha)
    T_rot_X[2, 2] = np.cos(alpha)
    T_rot_X[1, 2] = -np.sin(alpha)
    T_rot_X[2, 1] = np.sin(alpha)
    
    return T_rot_X

def Rot_Y(beta):
    """A function to return homogeneous rotation matrix along y-axis
    
    Arguments:
        beta {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Y = np.mat(np.eye(4,4))

    T_rot_Y[0, 0] = np.cos(beta)
    T_rot_Y[2, 2] = np.cos(beta)
    T_rot_Y[0, 2] = np.sin(beta)
    T_rot_Y[2, 0] = -np.sin(beta)
    
    return T_rot_Y

def Rot_Z(gamma):
    """A function to return homogeneous rotation matrix along z-axis
    
    Arguments:
        gamma {float} -- angle in radians
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous rotation matrix along y-axis
    """
    T_rot_Z = np.mat(np.eye(4,4))

    T_rot_Z[0, 0] = np.cos(gamma)
    T_rot_Z[1, 1] = np.cos(gamma)
    T_rot_Z[0, 1] = -np.sin(gamma)
    T_rot_Z[1, 0] = np.sin(gamma)

    return T_rot_Z

def Translation(axis, distance):
    """A funtion to return homegeneous translation matrix
    
    Arguments:
        axis {string} -- either 'x' or 'y' or 'z'
        distance {float} -- the distance to travel along the axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix
    """
    if axis in ['x', 'y', 'z']:
        if axis == 'x':
            return Trans_X(distance)
        elif axis == 'y':
            return Trans_Y(distance)
        else: # axis == 'z'
            return Trans_Z(distance)
    else:
        rospy.logerr('Axis wrong! Return identity matrix')
        return np.mat(np.eye(4,4))

def Trans_X(dist):
    """A funtion to return homogeneous translation matrix along x-axis
    
    Arguments:
        dist {float} -- distance to travel along x-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along x-axis
    """
    T_trans_X = np.mat(np.eye(4,4))

    T_trans_X[0, 3] = dist

    return T_trans_X

def Trans_Y(dist):
    """A funtion to return homogeneous translation matrix along y-axis
    
    Arguments:
        dist {float} -- distance to travel along y-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along y-axis
    """
    T_trans_Y = np.mat(np.eye(4,4))

    T_trans_Y[1, 3] = dist

    return T_trans_Y

def Trans_Z(dist):
    """A funtion to return homogeneous translation matrix along z-axis
    
    Arguments:
        dist {float} -- distance to travel along z-axis
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous translation matrix along z-axis
    """
    T_trans_Z = np.mat(np.eye(4,4))

    T_trans_Z[2, 3] = dist

    return T_trans_Z

def quaternion_to_rotation_matrix(quat):
    """A function to transform quaternion to homogeneous rotation matrix
    
    Arguments:
        rot {Quaternion} -- Quaternion-type msg
    
    Returns:
        Numpy matrix in 4x4  -- Result homogeneous rotation matrix from input quaternion
    """
    # get qx, qy, qz and qw from Quaternion-type msg
    qx = quat.x
    qy = quat.y
    qz = quat.z
    qw = quat.w

    rot_matrix = np.mat(np.eye(4))

    rot_matrix[0, 0] = 1-2*np.square(qy)-2*np.square(qz)
    rot_matrix[0, 1] = 2*qx*qy-2*qz*qw
    rot_matrix[0, 2] = 2*qx*qz+2*qy*qw
    rot_matrix[1, 0] = 2*qx*qy+2*qz*qw
    rot_matrix[1, 1] = 1-2*np.square(qx)-2*np.square(qz)
    rot_matrix[1, 2] = 2*qy*qz-2*qx*qw
    rot_matrix[2, 0] = 2*qx*qz-2*qy*qw
    rot_matrix[2, 1] = 2*qy*qz+2*qx*qw
    rot_matrix[2, 2] = 1-2*np.square(qx)-2*np.square(qy)
    
    return rot_matrix

def vector3_to_translation_matrix(vec3):
    """A function to transfrom from Vector3-type msg to homogeneous translation matrix
    
    Arguments:
        vec3 {Vector3} -- Vector3-type msg
    
    Returns:
        Numpy matrix in 4x4  -- Result homogeneous translation matrix from input vector3
    """
    
    # get x, y and z from Vector3-type msg
    x = vec3.x
    y = vec3.y
    z = vec3.z

    trans_matrix = np.mat(np.eye(4))
    trans_matrix[0, 3] = x
    trans_matrix[1, 3] = y
    trans_matrix[2, 3] = z

    return trans_matrix

def transform_to_matrix(tf):
    """A function to get homogeneous matrix from transform
    
    Arguments:
        tf {Transform} -- Tranform-type msg as given transform
    
    Returns:
        Numpy matrix in 4x4 -- 4x4 homogeneous matrix representing given transfrom
    """

    vec3 = tf.translation
    quat = tf.rotation

    trans_matrix = vector3_to_translation_matrix(vec3)
    rot_matrix = quaternion_to_rotation_matrix(quat)

    # because of the transformation from tf, multiply trans_matrix first
    com_matrix = trans_matrix*rot_matrix
    
    return com_matrix

def rpy_from_quaternion(quat):
    """A function to get roll, pitch, yaw respectively from quaternion

    Arguments:
        quat {Quaternion} -- a quaternion resulted from rotation first about x-axis, then about y-axis and finally about z-axis

    Returns:
        [tuple] -- a tuple containing roll, pitch, yaw respectively
    """

    T_rot = quaternion_to_rotation_matrix(quat)

    # alpha, beta, gamma are all restricted to (-pi/2, pi/2), so sin(beta) can be used to calculate beta
    # alpha: roll : rotation about x-axis
    # beta : pitch: rotation about y-axis
    # gamma: yaw  : rotation about z-axis

    alpha = np.arctan2(-T_rot[1, 2], T_rot[2, 2])
    
    beta = np.arcsin(T_rot[0, 2])
    
    gamma = np.arctan2(-T_rot[0, 1], T_rot[0, 0])

    return (alpha, beta, gamma)

def rpy_from_matrix(T_rot):
    """extract roll, pitch and yaw respectively from 

    Arguments:
        T_rot {4x4 Numpy matrix} -- 4x4 homogeneous transform matrix in SE3

    Returns:
        [tuple] -- a tuple containing roll, pitch, yaw respectively
    """
    
    # alpha, beta, gamma are all restricted to (-pi/2, pi/2), so sin(beta) can be used to calculate beta
    # alpha: roll : rotation about x-axis
    # beta : pitch: rotation about y-axis
    # gamma: yaw  : rotation about z-axis

    alpha = np.arctan2(-T_rot[1, 2], T_rot[2, 2])
    
    beta = np.arcsin(T_rot[0, 2])
    
    gamma = np.arctan2(-T_rot[0, 1], T_rot[0, 0])

    return (alpha, beta, gamma)

def quaternion_from_matrix(T_rot):
    
    R_rot = T_rot[0:3, 0:3]

    K2 = np.mat(np.zeros((4,4)))

    # 1st row
    K2[0, 0] = R_rot[0, 0] - R_rot[1,1]
    K2[0, 1] = R_rot[1, 0] + R_rot[0, 1]
    K2[0, 2] = R_rot[2, 0]
    K2[0, 3] = -R_rot[2, 1]

    # 2nd row
    K2[1, 0] = K2[0, 1]
    K2[1, 1] = R_rot[1, 1] - R_rot[0, 0]
    K2[1, 2] = R_rot[2, 1]
    K2[1, 3] = R_rot[2, 0]

    # 3rd row
    K2[2, 0] = K2[0, 2]
    K2[2, 1] = K2[1, 2]
    K2[2, 2] = -R_rot[0, 0] - R_rot[1, 1]
    K2[2, 3] = R_rot[0, 1] - R_rot[1, 0]

    # 4th row
    K2[3, 0] = K2[0, 3]
    K2[3, 1] = K2[1, 3]
    K2[3, 2] = K2[2, 3]
    K2[3, 3] = R_rot[0, 0] + R_rot[1, 1]

    # do NOT forget coefficient
    K2 = 0.5 * K2

    # calculate eigen values and eigen vectors of K2
    K2_eig = np.linalg.eig(K2)

    # eigen values
    eig_vals = K2_eig[0]

    # index of max value
    arg_max = eig_vals.argmax()
    
    eig_vec = K2_eig[1][:,arg_max]

    quat = Quaternion()

    quat.x = float(eig_vec[0])
    quat.y = float(eig_vec[1])
    quat.z = float(eig_vec[2])
    quat.w = -float(eig_vec[3])

    return quat
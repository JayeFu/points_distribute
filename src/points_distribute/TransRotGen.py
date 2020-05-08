#!/usr/bin/env python 

import numpy as np

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
#!/usr/bin/env python

import pandas as pd
import numpy as np

from geometry_msgs.msg import Transform

class SampleOptimizer:
    
    def __init__(self, room_dimension=(10.0, 6.0, 3.0)):
        
        # room dimension of simula
        self._room_dimension = room_dimension

        # weights of translation and rotation for traget funciton
        self._trans_weights = (1.0, 1.0, 1.0)
        self._rot_weights = (1.0, 1.0, 1.0)

        # range of deviation for sampling
        self._trans_range = (0.05, 0.05, 0.05)
        self._rot_range = (0.05, 0.05, 0.05)

        # a list to contain the realtive poses
        self._relative_pose_list = list()


    def set_trans_weight(self, trans_weights=(1.0, 1.0, 1.0)):
        """setter function for self._trans_weights

        Keyword Arguments:
            trans_weights {tuple} -- translation weights for target function (default: {(1.0, 1.0, 1.0)})
            [0]: mbx
            [1]: fwx
            [2]: uav
        """
        
        self._trans_weights = trans_weights

    def set_rot_weights(self, rot_weights=(1.0, 1.0, 1.0)):
        """setter function for self._rot_weights

        Keyword Arguments:
            rot_weights {tuple} -- rotation weights for target function (default: {(1.0, 1.0, 1.0)})
            [0]: mbx
            [1]: fwx
            [2]: uav
        """

        self._rot_weights = rot_weights

    def set_trans_range(self, trans_range=(0.05, 0.05, 0.05)):
        """setter function for self._trans_range

        Keyword Arguments:
            trans_range {tuple} -- range of deviation for translation in sampling (default: {(0.05, 0.05, 0.05)})
            [0]: x
            [1]: y
            [2]: z
        """
        
        self._trans_range = trans_range

    def set_rot_range(self, rot_range=(0.05, 0.05, 0.05)):
        """setter function for self._rot_range

        Keyword Arguments:
            rot_range {tuple} -- range of deviation for rotation in sampling (default: {(0.05, 0.05, 0.05)})
            [0]: alpha
            [1]: beta
            [2]: gamma
        """

        self._rot_range = rot_range

    def read_relative_poses(self, file_name="relative_poses.csv"):
        
        # read out the data from csv files
        csv_data = pd.read_csv(file_name) # in DataFrame

        # get ndarray from DataFrame
        relative_pose_array = csv_data.values # in ndarray

        # get shape, especially rows, of array for later iteration
        (rows, cols) = relative_pose_array.shape

        # iterate through array
        for row_idx in range(rows):
            # get the data of a certain time point
            time_slice = relative_pose_array[row_idx]

            time = time_slice[0]

            # transform from mbx to fwx
            m_to_f_tf = Transform()
            # set translation from mbx to fwx
            m_to_f_tf.translation.x = time_slice[1]
            m_to_f_tf.translation.y = time_slice[2]
            m_to_f_tf.translation.z = time_slice[3]
            # set rotation from mbx to fwx
            m_to_f_tf.rotation.x = time_slice[4]
            m_to_f_tf.rotation.y = time_slice[5]
            m_to_f_tf.rotation.z = time_slice[6]
            m_to_f_tf.rotation.w = time_slice[7]

            # transform from mbx to uav
            m_to_u_tf = Transform()
            # set translation from mbx to uav
            m_to_u_tf.translation.x = time_slice[8]
            m_to_u_tf.translation.y = time_slice[9]
            m_to_u_tf.translation.z = time_slice[10]
            # set rotation from mbx to fwx
            m_to_u_tf.rotation.x = time_slice[11]
            m_to_u_tf.rotation.y = time_slice[12]
            m_to_u_tf.rotation.z = time_slice[13]
            m_to_u_tf.rotation.w = time_slice[15]

            time_slice_tuple = (time, m_to_f_tf, m_to_u_tf)

            self._relative_pose_list.append(time_slice_tuple)
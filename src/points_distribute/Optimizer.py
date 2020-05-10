#!/usr/bin/env python

import pandas as pd
import numpy as np

import rospy

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

from points_distribute.TransRotGen import Rotation, Translation, rpy_from_matrix, transform_to_matrix


class SampleOptimizer:
    
    def __init__(self, room_dimension=(10.0, 6.0, 3.0)):
        
        # room dimension of simula
        self._room_dimension = room_dimension

        # bounding box dimension
        self._box_dimension = tuple()
        # all divided by 2
        # comma after float to indicate this is tuple
        self._box_dimension += (self._room_dimension[0]/2.0, ) # x-axis
        self._box_dimension += (self._room_dimension[1]/2.0, ) # y-axis
        self._box_dimension += (self._room_dimension[2]/2.0, ) # z-axis

        # weights of translation and rotation for traget funciton
        self._trans_weights = (1.0, 1.0, 1.0)
        self._rot_weights = (1.0, 1.0, 1.0)

        # range of deviation for sampling
        self._trans_range = (0.05, 0.05, 0.05)
        self._rot_range = (0.05, 0.05, 0.05)

        # a list to contain the realtive poses
        self._relative_pose_list = list()

        # default mbx pose
        # TODO: currently translation at origin, add initial offset
        self._T_o_to_m_default = Rotation('y', np.pi/4.0) * Rotation('z', -np.pi/2.0)

        # a list to contain allocated absolute poses
        self._absolute_poses_list = list()


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

    def compute_square_distance_to_plane(self, plane, T_o_to_x):
        """Funciton to calculate squared distance of the point in homogeneous transform matrix to two oppisite plane

        Arguments:
            plane {str} -- two planes to calculate distance, should only be one in front-back, left-right and up-down
            T_o_to_x {4x4 Numpy matrix} -- homogeneous transform matrix in SE3

        Returns:
            [float] -- squared distance
        """
        
        # extract item from pos_pt
        x = T_o_to_x[0, 3]
        y = T_o_to_x[1, 3]
        z = T_o_to_x[2, 3]

        # positive x y z positions of bounding box
        x_box = self._box_dimension[0]/2.0
        y_box = self._box_dimension[1]/2.0
        z_box = self._box_dimension[2]/2.0

        # squared distance
        sq_dist = 0.0
        
        if plane == 'front-back': # x-axis
            # front
            sq_dist += np.square(x-x_box)
            # back
            sq_dist += np.square(x+x_box)

        elif plane == 'left-right': # y-axis
            # left
            sq_dist += np.square(y+y_box)
            # right
            sq_dist += np.square(y-y_box)

        elif plane == 'up-down': # z-axis
            # up
            sq_dist += np.square(z-z_box)
            # down
            sq_dist += np.square(z+z_box)

        else: # other planes, error occurs
            print "ERROR! Wrong plane!"

        return sq_dist
        

    def compute_distance_cost_of_T(self, T_o_to_x):
        """Calculate distance cost of a certain position, containing squared distance to front-back, left-right and up-down

        Arguments:
            T_o_to_x {4x4 Numpy matrix} -- homogeneous transform matrix in SE3

        Returns:
            [float] -- distance cost to all six planes
        """
        
        # distance cost
        dist_cost = 0.0

        # front and back: x-axis
        dist_cost += self.compute_square_distance_to_plane('front-back', T_o_to_x)

        # left and right: y-axis
        dist_cost += self.compute_square_distance_to_plane('left-right', T_o_to_x)

        # up and down: z-axis
        dist_cost += self.compute_square_distance_to_plane('up-down', T_o_to_x)

        return dist_cost

    def compute_change_cost_of_T(self, T_o_to_x_prev, T_o_to_x_curr):
        """Calculate change cost of only one from transform matrix

        Arguments:
            T_o_to_x_prev {4x4 Numpy matrix} -- previous absolute homogeneous transform matrix in SE3 of x in mbx, fwx or uav
            T_o_to_x_curr {4x4 Numpy matrix} -- current absolute homogeneous transform matrix in SE3 of x in mbx, fwx or uav

        Returns:
            [float] -- change cost of x
        """
        
        # translation components in previous matrix
        x_prev = T_o_to_x_prev[0, 3]
        y_prev = T_o_to_x_prev[1, 3]
        z_prev = T_o_to_x_prev[2, 3]

        # rotation components in previous matrix
        (roll_prev, pitch_prev, yaw_prev) = rpy_from_matrix(T_o_to_x_prev)

        # translation components in current matrix
        x_curr = T_o_to_x_curr[0, 3]
        y_curr = T_o_to_x_curr[1, 3]
        z_curr = T_o_to_x_curr[2, 3]

        # rotation components in previous matrix
        (roll_current, pitch_current, yaw_current) = rpy_from_matrix(T_o_to_x_curr)

        # cost of translation
        cost_x = np.square(x_prev-x_curr)
        cost_y = np.square(y_prev-y_curr)
        cost_z = np.square(z_prev-z_curr)

        cost_trans = cost_x + cost_y + cost_z

        # cost of rotation
        cost_roll = np.square(roll_prev-roll_current)
        cost_pitch = np.square(pitch_prev-pitch_current)
        cost_yaw = np.square(yaw_prev-yaw_current)

        cost_rot = cost_roll + cost_pitch + cost_yaw

        # add all cost
        cost_tf_change = cost_trans + cost_rot

        return cost_tf_change

    def compute_distance_cost(self, T_o_to_tuple):
        """Calculate distance cost of all(mbx, fwx and uav), usually for current transform matrice

        Arguments:
            T_o_to_tuple {tuple} -- tuple of transform matrice, usually current transform matrice

        Returns:
            [float] -- distance cost of all
        """
        
        # extract matrix from tuple respectively
        T_o_to_m = T_o_to_tuple[0]
        T_o_to_f = T_o_to_tuple[1]
        T_o_to_u = T_o_to_tuple[2]

        # mbx distance cost
        dist_cost_m = self.compute_distance_cost_of_T(T_o_to_m)

        # fwx distance cost
        dist_cost_f = self.compute_distance_cost_of_T(T_o_to_f)

        # uav distance cost
        dist_cost_u = self.compute_distance_cost_of_T(T_o_to_u)

        # add all distance cost
        dist_cost_all = dist_cost_m + dist_cost_f + dist_cost_u

        return dist_cost_all

    def compute_change_cost(self, T_o_to_tuple_prev, T_o_to_tuple_curr):
        """Calculate change cost of all (mbx, fwx and uav)

        Arguments:
            T_o_to_tuple_prev {tuple} -- tuple of CURRENT absolute homogeneous transform matrice of all
            T_o_to_tuple_curr {tuple} -- tuple of PREVIOUS absolute homogeneous transform matrice of all

        Returns:
            [float] -- change cost of all
        """
        
        # extract previous matrice from tuple
        T_o_to_m_prev = T_o_to_tuple_prev[0]
        T_o_to_f_prev = T_o_to_tuple_prev[1]
        T_o_to_u_prev = T_o_to_tuple_prev[2]

        # extract current matrice from tuple
        T_o_to_m_curr = T_o_to_tuple_curr[0]
        T_o_to_f_curr = T_o_to_tuple_curr[1]
        T_o_to_u_curr = T_o_to_tuple_curr[2]

        # change cost of mbx
        change_cost_m = self.compute_change_cost_of_T(T_o_to_m_prev, T_o_to_m_curr)

        # change cost of fwx
        change_cost_f = self.compute_change_cost_of_T(T_o_to_f_prev, T_o_to_f_curr)

        # change cost of uav
        change_cost_u = self.compute_change_cost_of_T(T_o_to_u_prev, T_o_to_u_curr)

        # add all change cost together
        change_cost_all = change_cost_m + change_cost_f + change_cost_u

        return change_cost_all

    def compute_cost(self, T_o_to_tuple_prev, T_o_to_tuple_curr):
        """Calculate cost of a certain allocation of absolute pose with respective to previous allocation

        Arguments:
            T_o_to_tuple_prev {tuple} -- tuple of CURRENT absolute homogeneous transform matrice of all
            T_o_to_tuple_curr {tuple} -- tuple of PREVIOUS absolute homogeneous transform matrice of all

        Returns:
            [float] -- cost, or say, target function of a certain allocation of aboslute pose
        """

        # compute distance cost of current matrice
        dist_cost = self.compute_distance_cost(T_o_to_tuple_curr)

        # compute change cost between previous matrice and current matrices 
        change_cost = self.compute_change_cost(T_o_to_tuple_prev, T_o_to_tuple_curr)

        cost = dist_cost + change_cost

        return cost

    def allocate_first_pose(self):

        # get first time slice tuple
        first_time_slice_tuple = self._relative_pose_list[0]

        # get the time of first pose
        time = first_time_slice_tuple[0]

        # tf from mbx to fwx and uav
        m_to_f_tf = first_time_slice_tuple[1]
        m_to_u_tf = first_time_slice_tuple[2]

        # convert tf to matrix
        T_m_to_f = transform_to_matrix(m_to_f_tf)
        T_m_to_u = transform_to_matrix(m_to_u_tf)

        # generate nutation matrix

        # step for deflection against z-axis
        step_def = 1.0
        # range for deflection against z-axis
        range_def = 10.0
        # times for iteration in deflection
        times_def = int(range_def/step_def)

        # step for rotation about z-axis
        step_rot = 10.0
        # range for rotation about z-axis
        range_rot = 360.0
        # times for iteration in rotatoin
        times_rot = int(range_rot/step_rot)

        for counter_def in range(times_def+1): # include 10 degrees itself
            for counter_rot in range(times_rot): # do not need to include 360 degrees, since 0 degree = 360 degree
                # deflection angle
                angle_def = counter_def * step_def
                # rotation angle
                angle_rot = counter_rot * step_rot
                # nutation matrix
                T_m_to_nutation = Rotation('z', angle_rot/180.0*np.pi) * Rotation('x', angle_def/180.0*np.pi) * Rotation('z', -angle_rot/180.0*np.pi)
                
                # transform matrix to mbx
                T_o_to_nutation = self._T_o_to_m_default * T_m_to_nutation



    def allocate_next_pose(self):
        pass

    def allocate_other_poses(self):
        pass

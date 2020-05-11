#!/usr/bin/env python

import pandas as pd
import numpy as np

import rospy

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

from points_distribute.TransRotGen import Rotation, Translation, rpy_from_matrix, xyz_from_matrix, transform_to_matrix

import time

class SampleOptimizer:
    
    def __init__(self, room_dimension=(10.0, 6.0, 3.0)):
        """Construction function for SampleOptimizer

        Keyword Arguments:
            room_dimension {tuple} -- room dimension, [0]:x, [1]:y, [2]:z (default: {(10.0, 6.0, 3.0)})
        """
        
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
        self._trans_range = (0.03, 0.03, 0.03)
        self._rot_range = (0.03, 0.03, 0.03)

        # steps of deviation for sampling
        self._trans_step = (0.01, 0.01, 0.01)
        self._rot_step = (0.01, 0.01, 0.01)

        # times for iteration in sampling of translation
        self._trans_seq_list = list()
        
        for idx in range(3):
            start = -self._trans_range[idx]
            end = self._trans_range[idx]
            num = 2*int(self._trans_range[idx]/self._trans_step[idx]) + 1
            trans_seq = np.linspace(start, end, num=num)
            self._trans_seq_list.append(trans_seq)

        # print self._trans_times

        # times for iteration in sampling of rotation
        self._rot_seq_list = list()

        for idx in range(3):
            start = -self._rot_range[idx]
            end = self._rot_range[idx]
            num = 2*int(self._rot_range[idx]/self._rot_step[idx]) + 1
            rot_seq = np.linspace(start, end, num=num)
            self._rot_seq_list.append(rot_seq)

        # print self._rot_times

        # a list to contain the realtive poses
        self._relative_pose_list = list()

        # default mbx pose
        # TODO: currently translation at origin, add initial offset
        self._T_o_to_m_default = Translation('z', -0.2) * Rotation('y', np.pi/4.0) * Rotation('z', -np.pi/2.0)

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
        """Read relative poses between mbx, fwx and uav

        Keyword Arguments:
            file_name {str} -- csv file containing relative poses (default: {"relative_poses.csv"})
            index definition:
            [0]: time
            [1]-[3]: x,y,z between mbx and fwx
            [4]-[7]: qx,qy,qz,qw between mbx and fwx
            [8]-[10]: x,y,z between mbx and uav
            [11]-[14]: qx,qy,qz,qw between mbx and uav
        """
        
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

            happen_time = time_slice[0]

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
            m_to_u_tf.rotation.w = time_slice[14]

            time_slice_tuple = (happen_time, m_to_f_tf, m_to_u_tf)

            self._relative_pose_list.append(time_slice_tuple)

            # get number of relative poses
            # print len(self._relative_pose_list)

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
        """Allocate first pose from relative pose list to absolute pose list
        """

        print "Allocating 1st pose"

        # get first time slice tuple
        first_time_slice_tuple = self._relative_pose_list[0]

        # get the time of first pose
        happen_time = first_time_slice_tuple[0]

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

        # minimum cost for allocating first pose
        cost_min = 100.0
        # counter of both deflection and rotation at minimum cost
        counter_min = None # will be tuple
        # angle of both deflection and rotation at minimum cost
        angle_min = None # will be tuple
        # tuple of transform matrice of mbx, fwx and uav
        T_o_to_tuple_min = None # will be tuple

        # iterate for each deflection angle and nutation angle
        for counter_def in range(times_def+1): # include 10 degrees itself
            for counter_rot in range(times_rot): # do not need to include 360 degrees, since 0 degree = 360 degree
                # deflection angle
                angle_def = counter_def * step_def
                # rotation angle
                angle_rot = counter_rot * step_rot
                # nutation matrix
                T_m_with_nutation = Rotation('z', angle_rot/180.0*np.pi) * Rotation('x', angle_def/180.0*np.pi) * Rotation('z', -angle_rot/180.0*np.pi)
                
                # transform matrix to mbx
                T_o_to_m_with_nutation = self._T_o_to_m_default * T_m_with_nutation

                # calculate matrix for fwx and uav
                T_o_to_f = T_o_to_m_with_nutation * T_m_to_f
                T_o_to_u = T_o_to_m_with_nutation * T_m_to_u

                T_o_to_tuple = (T_o_to_m_with_nutation, T_o_to_f, T_o_to_u)

                # calculate cost of current poses
                cost = self.compute_distance_cost(T_o_to_tuple)

                # print "At (counter_def, counter_rot)=({}, {}), cost={}".format(counter_def, counter_rot, cost)

                # if cost less than least cost before
                if cost < cost_min:
                    # update cost, counter, angle and matrix
                    cost_min = cost
                    counter_min = (counter_def, counter_rot)
                    angle_min = (angle_def, angle_rot)
                    T_o_to_tuple_min = T_o_to_tuple

        # print "At (counter_def, counter_rot)=({}, {}), cost_min={}".format(counter_min[0], counter_min[1], cost_min)
        # print "Then (angle_def, angle_rot)=({}, {})".format(angle_min[0], angle_min[1])

        # append to relative pose list
        self._absolute_poses_list.append(T_o_to_tuple_min)

    def allocate_next_pose(self, num):
        """Allocate next pose from relative pose list to absolute pose list

        Arguments:
            num {int} -- index of relative pose to be allocated in list
        """

        print "Allocating {}th pose".format(num+1)

        # read out current tfs from mbx to fwx and from mbx to uav
        current_time_slice_tuple = self._relative_pose_list[0]

        # get the time of first pose
        happen_time = current_time_slice_tuple[0]

        # tf from mbx to fwx and uav
        m_to_f_tf = current_time_slice_tuple[1]
        m_to_u_tf = current_time_slice_tuple[2]

        # convert tf to matrix
        T_m_to_f = transform_to_matrix(m_to_f_tf)
        T_m_to_u = transform_to_matrix(m_to_u_tf)
        
        # get last absolute poses in history
        T_o_to_tuple_prev = self._absolute_poses_list[-1]

        # get last absolute pose of mbx
        T_o_to_m_prev = T_o_to_tuple_prev[0]
        # print T_o_to_m_prev

        # extract rpy and xyz from matrix for later add or minus
        (roll_m_prev, pitch_m_prev, yaw_m_prev) = rpy_from_matrix(T_o_to_m_prev)
        (x_m_prev, y_m_prev, z_m_prev) = xyz_from_matrix(T_o_to_m_prev)

        # T_m_trans = Translation('x', x_m_prev) * Translation('y', y_m_prev) * Translation('z', z_m_prev)
        # T_m_rot = Rotation('x', roll_m_prev) * Rotation('y', pitch_m_prev) * Rotation('z', yaw_m_prev)

        # print T_m_trans * T_m_rot

        # extract translation sequence from list
        X_seq = self._trans_seq_list[0]
        Y_seq = self._trans_seq_list[1]
        Z_seq = self._trans_seq_list[2]

        # extract rotation sequence from list
        Roll_seq = self._rot_seq_list[0]
        Pitch_seq = self._rot_seq_list[1]
        Yaw_seq = self._rot_seq_list[2]

        # since translation will only change the vector column, so calculate it first to reduce run time
        T_m_trans = Translation('x', x_m_prev) * Translation('y', y_m_prev) * Translation('z', z_m_prev)

        # minimum cost
        cost_min = 100.0
        # tuple to contain dX, dY, dZ, dRoll, dPitch, dYaw corresponding to minimum cost
        addon_min = None
        # tuple to contain absolute transform matrice for mbx, fwx and uav
        T_o_to_tuple_min = None

        # start sampling
        # print "Start iteration"
        # start = time.time()
        for dX in X_seq:
            for dY in Y_seq:
                for dZ in Z_seq:
                    for dRoll in Roll_seq:
                        for dPitch in Pitch_seq:
                            for dYaw in Yaw_seq:
                                # get translation matrix
                                T_m_trans[0, 3] = x_m_prev + dX
                                T_m_trans[1, 3] = y_m_prev + dY
                                T_m_trans[2, 3] = z_m_prev + dZ
                                # get rotation matrix
                                T_m_rot = Rotation('x', roll_m_prev+dRoll) * Rotation('y', pitch_m_prev+dPitch) * Rotation('z', yaw_m_prev+dYaw)
                                # get origin to mbx current matrix
                                T_o_to_m_curr = T_m_trans * T_m_rot
                                # get origin to mbx and fwx current matrix respectively
                                T_o_to_f_curr = T_o_to_m_curr * T_m_to_f
                                T_o_to_u_curr = T_o_to_m_curr * T_m_to_u
                                
                                # combine into tuple
                                T_o_to_tuple_curr = (T_o_to_m_curr, T_o_to_f_curr, T_o_to_u_curr)

                                # calculate cost for current abs poses
                                cost = self.compute_cost(T_o_to_tuple_prev, T_o_to_tuple_curr)
                                # print "cost={}".format(cost)

                                # find minimum cost
                                if cost < cost_min:
                                    # update if minimum in history larger than current cost
                                    cost_min = cost
                                    addon_min = (dX, dY, dZ, dRoll, dPitch, dYaw)
                                    T_o_to_tuple_min = T_o_to_tuple_curr
        # print "End iteration"
        # end = time.time()
        # print "Use time {}s".format(end-start)

        # print "At {}, cost_min={}".format(addon_min, cost_min)
        # print "Then, abs matrice: {}".format(T_o_to_tuple_min)

        self._absolute_poses_list.append(T_o_to_tuple_min)

    def allocate_other_poses(self):
        """Allocate other poses except for 1st pose by iterating through relative pose list
        """
        
        # number of time points in _relative_pose_list
        pt_num = len(self._relative_pose_list)

        # iterate through relative pose list except the first since it has been allocated through allocate_first_pose
        for pt_idx in range(pt_num-1):
            self.allocate_next_pose(pt_idx+1)

        print "Allocation finished"

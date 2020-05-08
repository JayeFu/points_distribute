#!/usr/bin/env python

import pandas as pd
import numpy as np

from geometry_msgs.msg import Transform

from points_distribute.TransRotGen import Rotation, Translation, rpy_from_quaternion, transform_to_matrix

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

    def compute_square_distance_to_plane(self, plane, pos_vec3):
        """Funciton to calculate squared distance of a Vector3-type msg to two oppisite plane

        Arguments:
            plane {str} -- two planes to calculate distance, should only be one in front-back, left-right and up-down
            pos_vec3 {Vector3} -- Vector3-type msg of a position

        Returns:
            [float] -- squared distance
        """
        
        # extract item from pos_pt
        x = pos_vec3.x
        y = pos_vec3.y
        z = pos_vec3.z

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
        

    def compute_distance_cost_of_vec3(self, pos_vec3):
        """Calculate distance cost of a certain position, containing squared distance to front-back, left-right and up-down

        Arguments:
            pos_vec3 {Vector3} -- Vector3-type msg of a position

        Returns:
            [float] -- distance cost to all six planes
        """
        
        # distance cost
        dist_cost = 0.0

        # front and back: x-axis
        dist_cost += self.compute_square_distance_to_plane('front-back', pos_vec3)

        # left and right: y-axis
        dist_cost += self.compute_square_distance_to_plane('left-right', pos_vec3)

        # up and down: z-axis
        dist_cost += self.compute_square_distance_to_plane('up-down', pos_vec3)

        return dist_cost

    def compute_change_cost_of_tf(self, tf_from_o1, tf_from_o2): # both are Transform-type msg
        
        # translation components of tf1
        x1 = tf_from_o1.translation.x
        y1 = tf_from_o1.translation.y
        z1 = tf_from_o1.translation.z

        # rotation components of tf1
        quat1 = tf_from_o1.rotation
        (roll1, pitch1, yaw1) = rpy_from_quaternion(quat1)

        # translation components of tf2
        x2 = tf_from_o2.translation.x
        y2 = tf_from_o2.translation.y
        z2 = tf_from_o2.translation.z

        # rotation components of tf2
        quat2 = tf_from_o2.rotation
        (roll2, pitch2, yaw2) = rpy_from_quaternion(quat1)

        # cost of translation
        cost_x = np.square(x1-x2)
        cost_y = np.square(y1-y2)
        cost_z = np.square(z1-z2)

        cost_trans = cost_x + cost_y + cost_z

        # cost of rotation
        cost_roll = np.square(roll1-roll2)
        cost_pitch = np.square(pitch1-pitch2)
        cost_yaw = np.square(yaw1-yaw2)

        cost_rot = cost_roll + cost_pitch + cost_yaw

        # add all cost
        cost_tf_change = cost_trans + cost_rot

        return cost_tf_change

    def compute_distance_cost(self, pos_vec3_tuple):
        
        # mbx position vector3
        pos_vec3_m = pos_vec3_tuple[0]

        # fwx position vector3
        pos_vec3_f = pos_vec3_tuple[1]

        # uav position vector3
        pos_vec3_u = pos_vec3_tuple[2]

        # mbx distance cost
        dist_cost_m = self.compute_distance_cost_of_vec3(pos_vec3_m)

        # fwx distance cost
        dist_cost_f = self.compute_distance_cost_of_vec3(pos_vec3_f)

        # uav distance cost
        dist_cost_u = self.compute_distance_cost_of_vec3(pos_vec3_u)

        # add all distance cost
        dist_cost_all = dist_cost_m + dist_cost_f + dist_cost_u

        return dist_cost_all

    def compute_change_cost(self, tf_from_o_tuple1, tf_from_o_tuple2):
        
        # extract tf1s from tuple
        tf_from_o_to_m1 = tf_from_o_tuple1[0]
        tf_from_o_to_f1 = tf_from_o_tuple1[1]
        tf_from_o_to_u1 = tf_from_o_tuple1[2]

        # extract tf2s from tuple
        tf_from_o_to_m2 = tf_from_o_tuple2[0]
        tf_from_o_to_f2 = tf_from_o_tuple2[1]
        tf_from_o_to_u2 = tf_from_o_tuple2[2]

        # change cost of mbx
        change_cost_m = self.compute_change_cost_of_tf(tf_from_o_to_m1, tf_from_o_to_m2)

        # change cost of fwx
        change_cost_f = self.compute_change_cost_of_tf(tf_from_o_to_f1, tf_from_o_to_f2)

        # change cost of uav
        change_cost_u = self.compute_change_cost_of_tf(tf_from_o_to_u1, tf_from_o_to_u2)

        # add all change cost together
        change_cost_all = change_cost_m + change_cost_f + change_cost_u

        return change_cost_all

    def compute_cost(self, tf_from_o_tuple_before, tf_from_o_tuple_current):

        # construct pos_vec3_tuple
        pos_vec3_m = tf_from_o_tuple_current[0].translation
        pos_vec3_f = tf_from_o_tuple_current[1].translation
        pos_vec3_u = tf_from_o_tuple_current[0].translation

        pos_vec3_tuple = (pos_vec3_m, pos_vec3_f, pos_vec3_u)

        # compute distance cost of current tfs
        dist_cost = self.compute_distance_cost(pos_vec3_tuple)

        # compute change cost between previous tfs and current tfs 
        change_cost = self.compute_change_cost(tf_from_o_tuple_before, tf_from_o_tuple_current)

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



    def allocate_next_pose(self):
        pass

    def allocate_other_poses(self):
        pass

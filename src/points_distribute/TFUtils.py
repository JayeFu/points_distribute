#!/usr/bin/env python 

import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

from points_distribute.TransRotGen import quaternion_from_matrix

class TFBroadcaster:
    
    def __init__(self):
        
        # non-static broadcaster
        self._broadcaster = tf2_ros.TransformBroadcaster()

    def sendTransform(self, parent_link, child_link, T_parent_to_child):
        
        # first construct a tf
        parent_to_child_tf = TransformStamped()

        # some settings about tf
        parent_to_child_tf.header.stamp = rospy.Time.now()
        parent_to_child_tf.header.frame_id = parent_link
        parent_to_child_tf.child_frame_id = child_link

        # convert matrix to quaternion
        parent_to_child_tf.transform.rotation = quaternion_from_matrix(T_parent_to_child)

        self._broadcaster.sendTransform(parent_to_child_tf)
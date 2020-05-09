#!/usr/bin/env python

import numpy as np

import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

from points_distribute.TransRotGen import quaternion_from_matrix, Rotation, Translation

if __name__ == "__main__":
    
    # construct a node
    rospy.init_node('nutation_tf')

    # static broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # non-static broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    # transform from world to wx
    world_to_wx_tf = TransformStamped()

    # settings about world_to_wx_tf
    world_to_wx_tf.header.stamp = rospy.Time.now()
    world_to_wx_tf.header.frame_id = 'world'
    world_to_wx_tf.child_frame_id = 'wx_link'

    world_to_wx_tf.transform.rotation.w = 1.0

    static_broadcaster.sendTransform(world_to_wx_tf)

    rospy.loginfo("sent tf from world to wx")

    # rate in the loop
    rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        # transform matrix from wx to nutation
        T_wx_to_nutation = Rotation('x', 5.0/180.0*np.pi)

        print T_wx_to_nutation

        quat_wx_to_nutation = quaternion_from_matrix(T_wx_to_nutation)

        print quat_wx_to_nutation

        # transform from wx to nutation
        wx_to_nutation_tf = TransformStamped()

        # some settings about wx_to_nutation_tf
        wx_to_nutation_tf.header.stamp = rospy.Time.now()
        wx_to_nutation_tf.header.frame_id = 'wx_link'
        wx_to_nutation_tf.child_frame_id = 'nutation_link'

        # convert matrix to quaternion
        wx_to_nutation_tf.transform.rotation = quat_wx_to_nutation

        broadcaster.sendTransform(wx_to_nutation_tf)

        rate.sleep()


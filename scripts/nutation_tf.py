#!/usr/bin/env python

import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == "__main__":
    
    # construct a node
    rospy.init_node('nutation_tf')

    # static broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    # transform from world to wx
    world_to_wx_tf = TransformStamped()

    world_to_wx_tf.header.stamp = rospy.Time.now()
    world_to_wx_tf.header.frame_id = 'world'
    world_to_wx_tf.child_frame_id = 'wx_link'

    world_to_wx_tf.transform.rotation.w = 1.0

    static_broadcaster.sendTransform(world_to_wx_tf)

    rospy.loginfo("sent tf from world to wx")

    rospy.spin()

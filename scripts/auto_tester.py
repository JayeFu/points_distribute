#!/usr/bin/env python

import rospy

from points_distribute.Optimizer import SampleOptimizer

from geometry_msgs.msg import Transform, Vector3, Quaternion

if __name__ ==  "__main__":

    # construct a node
    rospy.init_node('auto_tester')

    optimizer  = SampleOptimizer()

    optimizer.allocate_first_pose()

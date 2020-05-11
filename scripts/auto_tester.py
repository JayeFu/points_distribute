#!/usr/bin/env python

import rospy

from points_distribute.Optimizer import SampleOptimizer

from geometry_msgs.msg import Transform, Vector3, Quaternion

if __name__ ==  "__main__":

    # construct a node
    rospy.init_node('auto_tester')

    # construct an optimizer
    optimizer  = SampleOptimizer()

    # read relative poses from file
    optimizer.read_relative_poses()

    # allocate first pose
    optimizer.allocate_first_pose()

    # testing with allocate next pose
    # optimizer.allocate_next_pose(num=1)

    # allocate other poses except the first one
    optimizer.allocate_other_poses()

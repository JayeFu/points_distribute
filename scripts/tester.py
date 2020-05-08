#!/usr/bin/env python

from points_distribute.Optimizer import SampleOptimizer

from geometry_msgs.msg import Vector3

if __name__ ==  "__main__":

    optimizer  = SampleOptimizer()

    new_pt = Vector3(0, 0, 0)

    print optimizer.compute_distance_cost_of_point(new_pt)
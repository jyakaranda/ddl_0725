#!/usr/bin/env python

import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped

from utils import LineTrajectory


class BuildTrajectory(object):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """
    def __init__(self):
        self.save_path = os.path.join(rospy.get_param("~save_path"), time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj")
        self.trajectory = LineTrajectory("/built_trajectory")
        # receive either goal points or clicked points
        self.publish_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_callback, queue_size=1)
        self.nav_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.clicked_point_callback, queue_size=1)
        # save the built trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

    def clicked_point_callback(self, msg):
        if isinstance(msg, PointStamped):
            self.trajectory.addPoint(msg.point)
        elif isinstance(msg, PoseStamped):
            self.trajectory.addPoint(msg.pose.position)

        # publish visualization of the path being built
        self.trajectory.publish_viz()

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)

if __name__=="__main__":
    rospy.init_node("build_trajectory")
    pf = BuildTrajectory()
    rospy.spin()

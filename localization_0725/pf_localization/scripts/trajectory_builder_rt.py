#!/usr/bin/env python

import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry

from utils import LineTrajectory, make_header, quaternion_to_angle


class BuildTrajectory(object):
    """ Listens for points published by RViz and uses them to build a trajectory. Saves the output to the file system.
    """

    def __init__(self):
        self.save_path = os.path.join(
            rospy.get_param(
                "~save_path",
                default=
                "/home/nvidia/workspace/catkin_ws/src/ddl_0725/pf_localization/trajectories"
            ),
            time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj")
        self.trajectory = LineTrajectory("/built_trajectory")
        self.odom_topic = rospy.get_param("~odom_topic", default="/pf/odom")

        self.sub_odom = rospy.Subscriber(
            self.odom_topic, Odometry, self.odomCB, queue_size=2)

        self.sub_pose = rospy.Subscriber("/current_pose", PoseStamped, self.poseCB, queue_size=1)

        # save the built trajectory on shutdown
        rospy.on_shutdown(self.saveTrajectory)

    def odomCB(self, msg):
        self.trajectory.addPoint(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            quaternion_to_angle(msg.pose.pose.orientation))
        self.trajectory.publish_viz()
    
    def poseCB(self, msg):
        self.trajectory.addPoint(
            msg.pose.position.x, msg.pose.position.y,
            quaternion_to_angle(msg.pose.orientation))
        self.trajectory.publish_viz()        

    def saveTrajectory(self):
        self.trajectory.save(self.save_path)


if __name__ == "__main__":
    rospy.init_node("build_trajectory_rt")
    print 'building trajectory.'
    pf = BuildTrajectory()
    rospy.spin()

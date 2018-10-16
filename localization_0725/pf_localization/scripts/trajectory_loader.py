#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path
import time, os
from utils import LineTrajectory
from localization_0725.srv import *


class LoadTrajectory(object):
    """ Loads a trajectory from the file system and publishes it to a ROS topic.
	"""

    def __init__(self):
        self.path = rospy.get_param(
            "~trajectory",
            default=
            "/home/nvidia/workspace/catkin_ws/src/ddl_0725/localization_0725/trajectories/2018-08-09-16-16-58.traj"
        )
        self.should_publish = bool(rospy.get_param("~publish", default=True))
        self.pub_topic = rospy.get_param(
            "~topic", default="/loaded_trajectory/recorded_path")

        # initialize and load the trajectory
        self.trajectory = LineTrajectory("/loaded_trajectory")
        self.trajectory.load(self.path)

        if self.should_publish:
            self.traj_pub = rospy.Publisher(self.pub_topic, Path, queue_size=1)

        # need to wait a short period of time before publishing  the first message
        time.sleep(0.5)

        # visualize the loaded trajectory for 5 seconds
        self.trajectory.publish_viz(duration=3.0)
        self.duration = rospy.Duration(1)

        self.srv_get_raw_path_ = rospy.Service('/get_raw_path', GetRawPath,
                                               self.handle_get_raw_path)
        # send the trajectory
        if self.should_publish:
            self.publish_trajectory()

    def publish_trajectory(self):
        print "Publishing trajectory to:", self.pub_topic
        while not rospy.is_shutdown():
            self.traj_pub.publish(self.trajectory.toPath())
            time.sleep(self.duration.to_sec())

    def handle_get_raw_path(self, req):
        if not isinstance(self.trajectory, LineTrajectory):
            self.trajectory = LineTrajectory("/loaded_trajectory")
            self.trajectory.laod(self.path)
        return GetRawPathResponse(self.trajectory.toPath())


if __name__ == "__main__":
    rospy.init_node("load_trajectory")
    pf = LoadTrajectory()
    if bool(rospy.get_param("~spin", default=True)):
        rospy.spin()

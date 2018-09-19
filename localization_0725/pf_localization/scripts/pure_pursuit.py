#!/usr/bin/env python
# -*- coding=utf-8 -*-

import time

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

import utils


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.

	    Set point determined with the method described here: 
	        http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
	    Relies on localization for ground truth vehicle position.
		"""

    def __init__(self):
        self.lookahead = rospy.get_param("~lookahead")
        self.max_reacquire = rospy.get_param("~max_reacquire")
        self.speed = float(rospy.get_param("~speed"))
        self.wrap = bool(rospy.get_param("~wrap"))
        wheelbase_length = float(rospy.get_param("~wheelbase"))

        self.trajectory = utils.LineTrajectory("/followed_trajectory")
        self.model = utils.AckermannModel(wheelbase_length)
        self.do_viz = True
        self.odom_timer = utils.Timer(10)
        self.iters = 0

        self.nearest_point = None
        self.lookahead_point = None

        self.tau_p = rospy.get_param('~tau_p', default=0.1)
        self.tau_i = rospy.get_param('~tau_i', default=0.1)
        self.tau_d = rospy.get_param('~tau_d', default=0.1)

        # set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        self.viz_namespace = "/pure_pursuit"
        self.nearest_point_pub = rospy.Publisher(
            self.viz_namespace + "/nearest_point", Marker, queue_size=1)
        self.lookahead_point_pub = rospy.Publisher(
            self.viz_namespace + "/lookahead_point", Marker, queue_size=1)

        # topic to send drive commands to
        self.control_pub = rospy.Publisher(
            "/ackermann_cmd_mux", AckermannDriveStamped, queue_size=1)

        # topic to listen for trajectories
        self.traj_sub = rospy.Subscriber(
            "/trajectory", Path, self.trajectory_callback, queue_size=1)

        # topic to listen for odometry messages, either from particle filter or the simulator
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, self.odom_callback, queue_size=1)
        print "Initialized. Waiting on messages..."

    def visualize(self):
        ''' Publishes visualization topics:
		       - Circle to indicate the nearest point along the trajectory
		       - Circle to indicate the chosen lookahead point
		'''
        if not self.do_viz:
            return
        # visualize: pure pursuit circle, lookahead intersection, lookahead radius line, nearest point
        if self.nearest_point_pub.get_num_connections() > 0 and isinstance(
                self.nearest_point, np.ndarray):
            self.nearest_point_pub.publish(
                utils.make_circle_marker(self.nearest_point, 0.5,
                                         [0.0, 0.0, 1.0], "/map",
                                         self.viz_namespace, 0, 3))

        if self.lookahead_point_pub.get_num_connections() > 0 and isinstance(
                self.lookahead_point, np.ndarray):
            self.lookahead_point_pub.publish(
                utils.make_circle_marker(self.lookahead_point, 0.5,
                                         [1.0, 1.0, 1.0], "/map",
                                         self.viz_namespace, 1, 3))

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
		'''
        print "Receiving new trajectory:", len(msg.polygon.points), "points"
        self.trajectory.clear()
        self.trajectory.fromPath(msg)
        self.trajectory.publish_viz(duration=0.0)

    def odom_callback(self, msg):
        ''' Extracts robot state information from the message, and executes pure pursuit control.
		'''
        pose = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            utils.quaternion_to_angle(msg.pose.pose.orientation)
        ])
        self.pure_pursuit(pose)

        # this is for timing info
        self.odom_timer.tick()
        self.iters += 1
        if self.iters % 20 == 0:
            print "Control fps:", self.odom_timer.fps()

    def pure_pursuit(self, pose):
        ''' Determines and applies Pure Pursuit control law

		    1. Find the nearest point on the trajectory
		    2. Traverse the trajectory looking for the nearest point that is the lookahead distance away from the 
		       car, and further along the path than the nearest point from step (1). This is the lookahead point.
		    3. Determine steering angle necessary to travel to the lookahead point from step (2)
		    4. Send the desired speed and steering angle commands to the robot

		    Special cases:
		        - If nearest_point is beyond the max path reacquisition distance, stop
		        - If nearest_point is between max reacquisition dist and lookahead dist, navigate to nearest_point
		        - If nearest_point is less than the lookahead distance, find the lookahead point as normal
		'''
        # stop if no trajectory has been received
        if self.trajectory.empty():
            return self.stop()

        # this instructs the trajectory to convert the list of waypoints into a numpy matrix
        if self.trajectory.dirty():
            self.trajectory.make_np_array()

        # step 1
        nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(
            pose[:2], self.trajectory.np_points)
        self.nearest_point = nearest_point

        if nearest_dist < self.lookahead:
            # step 2
            lookahead_point, i2, t2 = \
             utils.first_point_on_trajectory_intersecting_circle(pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
            if i2 == None:
                if self.iters % 5 == 0:
                    print "Could not find intersection, end of path?"
                self.lookahead_point = None
            else:
                if self.iters % 5 == 0:
                    print "found lookahead point"
                self.lookahead_point = lookahead_point
        elif nearest_dist < self.max_reacquire:
            if self.iters % 5 == 0:
                print "Reacquiring trajectory"
            self.lookahead_point = self.nearest_point
        else:
            self.lookahead_point = None

        # stop of there is no navigation target, otherwise use ackermann geometry to navigate there
        if not isinstance(self.lookahead_point, np.ndarray):
            self.stop()
        else:
            steering_angle = self.determine_steering_angle(
                pose, self.lookahead_point)
            # send the control commands
            self.apply_control(self.speed, steering_angle)

        self.visualize()

    def determine_steering_angle(self, pose, lookahead_point):
        ''' Given a robot pose, and a lookahead point, determine the open loop control 
		    necessary to navigate to that lookahead point. Uses Ackermann steering geometry.
		'''
        # get the lookahead point in the coordinate frame of the car
        # TODO: 可能需要修改正负号
        rot = utils.rotation_matrix(-pose[2])
        delta = np.array([lookahead_point - pose[0:2]]).transpose()
        local_delta = (rot * delta).transpose()
        local_delta = np.array([local_delta[0, 0], local_delta[0, 1]])
        # use the ackermann model
        steering_angle = self.model.steering_angle(local_delta)
        return steering_angle

    def apply_control(self, speed, steering_angle):
        self.actual_speed = speed
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg_stamped.header = utils.make_header("/base_link")
        drive_msg = AckermannDrive()
        drive_msg.speed = speed
        # 正为左转，负为右转
        drive_msg.steering_angle = steering_angle
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.control_pub.publish(drive_msg_stamped)

    def stop(self):
        print "Stopping"
        drive_msg_stamped = AckermannDriveStamped()
        drive_msg_stamped.header = utils.make_header("/base_link")
        drive_msg = AckermannDrive()
        drive_msg.speed = 0
        drive_msg.steering_angle = 0
        drive_msg.acceleration = 0
        drive_msg.jerk = 0
        drive_msg.steering_angle_velocity = 0
        drive_msg_stamped.drive = drive_msg
        self.control_pub.publish(drive_msg_stamped)


if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    print "Start pure pursuit..."
    pf = PurePursuit()
    rospy.spin()

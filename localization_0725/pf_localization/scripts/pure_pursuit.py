#!/usr/bin/env python
# -*- coding=utf-8 -*-

import time

import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav_msgs.srv import GetMap
from pf_localization.srv import GetRawPath, SmoothPath
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, PoseStamped, Twist, PolygonStamped
from visualization_msgs.msg import Marker
from copy import deepcopy

import utils


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.

        Set point determined with the method described here:
            http://www.ri.cmu.edu/pub_files/pub4/howard_thomas_2006_1/howard_thomas_2006_1.pdf
        Relies on localization for ground truth vehicle position.
        """

    def __init__(self):
        self.lookahead = rospy.get_param("~lookahead", default=1.5)
        self.max_reacquire = rospy.get_param("~max_reacquire", default=3.0)
        self.speed = float(rospy.get_param("~speed", default=1.5))
        self.wrap = bool(rospy.get_param("~wrap", default=0))
        wheelbase_length = float(rospy.get_param("~wheelbase", default=0.335))
        self.max_twiddle_step = rospy.get_param(
            "~max_twiddle_step", default=500)
        self.viz = rospy.get_param("~viz", default=True)
        self.margin = rospy.get_param("~margin", default=0.3)
        self.max_angle = rospy.get_param("~max_angle", default=0.4538)
        self.need_smooth = rospy.get_param("~need_smooth", default=True)
        self.angle_noise = rospy.get_param("~angle_noise", default=0.03)
        self.load_or_not = rospy.get_param("~load_or_not", default=False)

        self.trajectory = utils.LineTrajectory("/followed_trajectory")
        self.model = utils.AckermannModel(wheelbase_length)
        self.do_viz = True
        self.odom_timer = utils.Timer(10)
        self.iters = 0

        self.nearest_point = None
        self.lookahead_point = None

        self.tau_p = rospy.get_param('~tau_p', default=1.0)
        self.tau_i = rospy.get_param('~tau_i', default=0.0)
        self.tau_d = rospy.get_param('~tau_d', default=0.0)
        self.cte = 0.
        self.diff_cte = 0.
        self.int_cte = 0.
        self.prev_angle = 0.
        self.first_angle = True
        self.already_twiddled = True

        # set up the visualization topic to show the nearest point on the trajectory, and the lookahead point
        self.viz_namespace = "/pure_pursuit"
        self.nearest_point_pub = rospy.Publisher(
            self.viz_namespace + "/nearest_point", Marker, queue_size=1)
        self.lookahead_point_pub = rospy.Publisher(
            self.viz_namespace + "/lookahead_point", Marker, queue_size=1)

        self.pub_smoothed_path_ = rospy.Publisher(
            '/smoothed_path', Path, queue_size=2)

        # topic to send drive commands to
        self.control_pub = rospy.Publisher(
            "/ackermann_cmd_mux", AckermannDriveStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # topic to listen for odometry messages, either from particle filter or the simulator
        self.odom_sub = rospy.Subscriber(
            "/odom", Odometry, self.odom_callback, queue_size=1)

        self.cur_pose_sub = rospy.Subscriber(
            "/current_pose", PoseStamped, self.cur_pose_callback, queue_size=1)

        #self.pose_sub = rospy.Subscriber(
        #    "/initialpose",
        #    PoseWithCovarianceStamped,
        #    self.poseCB,
        #    queue_size=1)

        if self.viz:
            self.pub_move_point = rospy.Publisher(
                "/move_point", PoseArray, queue_size=1)
            self.pub_move_point_1 = rospy.Publisher(
                "/move_point/no_pid", PoseArray, queue_size=1)

        self.map_data = None
#        rospy.wait_for_service('/static_map')
#        try:
#            get_map = rospy.ServiceProxy('/static_map', GetMap)
#            resp = get_map()
#            self.map_data = resp.map
#        except rospy.ServiceException, e:
#            print "Service call failed: ", e

        if self.load_or_not:
            rospy.wait_for_service('/get_raw_path')
            try:
                get_raw_path = rospy.ServiceProxy('/get_raw_path', GetRawPath)
                resp = get_raw_path()
                raw_path = resp.raw_path
                self.trajectory.clear()
                self.trajectory.fromPath(raw_path)
                self.trajectory.publish_viz(duration=0.0)
                print "Get path."
            except rospy.ServiceException, e:
                print "Service call failed: ", e
        else:
            # topic to listen for trajectories
            self.traj_sub = rospy.Subscriber(
                "/trajectory", Path, self.trajectory_callback, queue_size=1)

        # if self.need_smooth:
        #     rospy.wait_for_service('/smooth_path')
        #     try:
        #         smooth_path = rospy.ServiceProxy('/smooth_path', SmoothPath)
        #         resp = smooth_path(self.trajectory.toPath())
        #         self.trajectory.clear()
        #         self.trajectory.fromPath(resp.smoothed_path)
        #         self.trajectory.publish_viz(duration=0.0)
        #         print "Get smoothed path."
        #     except rospy.ServiceException, e:
        #         print "Service call failed: ", e

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
        self.trajectory.clear()
        if isinstance(msg, PolygonStamped):
            print "Receiving new trajectory:", len(
                msg.polygon.points), "points"
            self.trajectory.fromPolygon(msg.polygon)
        elif isinstance(msg, Path):
            print "Receiving new trajectory:", len(msg.poses), "points"
            self.trajectory.fromPath(msg)
            return
        else:
            print "error trajectory!"
            return
        self.trajectory.publish_viz(duration=0.0)
        rospy.wait_for_service("/smooth_path")
        smoothed_path_ = None
        try:
            ''' 1. Smooth path by gradient descending.
                2. Remove point from smoothed path which is too close to its neighborhood points ,or its neighborhood points is too close(which means there is probably a peak in path).
                3. Publish the result path
            '''
            smooth_path_srv = rospy.ServiceProxy('/smooth_path', SmoothPath)
            resp = smooth_path_srv(self.trajectory.toPath())
            smoothed_path_ = resp.smoothed_path
            smoothed_path_.header = utils.make_header('/map')
            self.trajectory.clear()
            self.trajectory.fromPath(smoothed_path_)
            self.trajectory.publish_viz(duration=0.0)
        except rospy.ServiceException, e:
            print 'Service call failed: ', e
            return
        self.pub_smoothed_path_.publish(smoothed_path_)
        print "path smoothed"

    def odom_callback(self, msg):
        ''' Extracts robot state information from the message, and executes pure pursuit control.
        '''
        pose = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            utils.quaternion_to_angle(msg.pose.pose.orientation)
        ])

        if not self.already_twiddled and not self.trajectory.empty():
            pid, err = self.twiddle(pose)
            self.tau_p = pid[0]
            self.tau_d = pid[2]
            self.tau_i = pid[1]
            print "PID: ", pid, " err: ", err
            self.already_twiddled = True

        self.pure_pursuit(pose)

        # this is for timing info
        self.odom_timer.tick()
        self.iters += 1
        if self.iters % 20 == 0:
            print "Control fps:", self.odom_timer.fps()

    def cur_pose_callback(self, msg):
        ''' Extracts robot state information from the message, and executes pure pursuit control.
        '''
        pose = np.array([
            msg.pose.position.x, msg.pose.position.y,
            utils.quaternion_to_angle(msg.pose.orientation)
        ])

        if not self.already_twiddled and not self.trajectory.empty():
            pid, err = self.twiddle(pose)
            self.tau_p = pid[0]
            self.tau_d = pid[2]
            self.tau_i = pid[1]
            print "PID: ", pid, " err: ", err
            self.already_twiddled = True

        self.pure_pursuit(pose)

        # this is for timing info
        self.odom_timer.tick()
        self.iters += 1
        if self.iters % 20 == 0:
            print "Control fps:", self.odom_timer.fps()

    def mapCB(self, msg):
        if isinstance(self.map_data, OccupancyGrid):
            return
        self.map_data = msg

    def poseCB(self, msg):
        ''' Get the start pose, twiddle the PID params.
        '''
        print "Start twiddle..."
        pose = [
            msg.pose.position.x, msg.pose.position.y,
            utils.quaternion_to_angle(msg.pose.orientation)
        ]
        err = self.measurement([1., 0., 0.], pose, self.pub_move_point_1,
                               100000.)
        print "without pid, err:", err
        pid, err = self.twiddle(pose)
        self.tau_p = pid[0]
        self.tau_d = pid[2]
        self.tau_i = pid[1]
        self.first_angle = True
        self.measurement(pid, pose, self.pub_move_point, 100000.)
        print "PID:", pid[0], pid[1], pid[2], "err:", err

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
            print "empty traj."
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
            # TODO: PID control
            steering_angle = self.determine_steering_angle(
                pose, self.lookahead_point)

            if self.first_angle:
                self.prev_angle = steering_angle
                self.int_cte = 0.
                self.first_angle = False

            self.diff_cte = steering_angle - self.prev_angle
            pid_angle = self.tau_p * steering_angle + self.tau_d * self.diff_cte + self.tau_i * self.int_cte
            self.int_cte += pid_angle
            self.prev_angle = pid_angle
            # send the control commands
            # self.apply_control(self.speed, steering_angle)
            self.apply_control(self.speed, pid_angle)

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
        # drive_msg_stamped = AckermannDriveStamped()
        # drive_msg_stamped.header = utils.make_header("/base_link")
        # drive_msg = AckermannDrive()
        # drive_msg.speed = speed
        # # 正为左转，负为右转
        # drive_msg.steering_angle = steering_angle
        # drive_msg.acceleration = 0
        # drive_msg.jerk = 0
        # drive_msg.steering_angle_velocity = 0
        # drive_msg_stamped.drive = drive_msg
        # self.control_pub.publish(drive_msg_stamped)
        vel = Twist()
        vel.linear.x = speed
        vel.angular.z = steering_angle
        self.cmd_pub.publish(vel)

    def stop(self):
        print "Stopping"
        # drive_msg_stamped = AckermannDriveStamped()
        # drive_msg_stamped.header = utils.make_header("/base_link")
        # drive_msg = AckermannDrive()
        # drive_msg.speed = 0
        # drive_msg.steering_angle = 0
        # drive_msg.acceleration = 0
        # drive_msg.jerk = 0
        # drive_msg.steering_angle_velocity = 0
        # drive_msg_stamped.drive = drive_msg
        # self.control_pub.publish(drive_msg_stamped)
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.cmd_pub.publish(vel)

    def twiddle(
            self,
            pose,
            tol=0.01,
    ):
        ''' Find the local optimal PID params by twiddle.
            ref: Udacity Artificial Intelligence Robotics: less 5 PID control
        '''
        p = [0., 0., 0.]
        dp = [1., 1., 1.]
        best_err = 10000.
        step = 0

        while sum(dp) > tol and step < self.max_twiddle_step:
            for i in range(len(p)):
                p[i] += dp[i]
                err = self.measurement(p, pose, self.pub_move_point, best_err)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] -= 2 * dp[i]
                    err = self.measurement(p, pose, self.pub_move_point,
                                           best_err)
                    if err < best_err:
                        best_err = err
                        dp[i] *= 1.1
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.9
            step += 1
        print "twiddle steps:", step
        return p, best_err

    def measurement(self, pid, pose, pub, best_err=100000.):
        ''' Calculate the error in path pursuit, given the PID params.
            In other words, check the smoothness of the pursuit process.
        '''
        if self.trajectory.empty():
            print 'no trajectory found'
            return self.stop()
        # print "Start measurement:", pid
        # this instructs the trajectory to convert the list of waypoints into a numpy matrix
        if self.trajectory.dirty():
            self.trajectory.make_np_array()

        lookahead_point = np.ndarray(1)
        current_pose = deepcopy(pose)
        first_angle = True
        prev_angle = 0.
        int_cte = 0.
        diff_cte = 0.
        err = 0.
        pa = PoseArray()
        pa.header = utils.make_header("/map")
        step = 0
        while isinstance(lookahead_point, np.ndarray) and err < best_err:
            pre_pose = current_pose
            # step 1
            nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(
                current_pose[:2], self.trajectory.np_points)

            if nearest_dist < self.lookahead:
                # step 2
                lookahead_point, i2, t2 = \
                 utils.first_point_on_trajectory_intersecting_circle(current_pose[:2], self.lookahead, self.trajectory.np_points, i+t, wrap=self.wrap)
                if i2 == None:
                    lookahead_point = None
            elif nearest_dist < self.max_reacquire:
                lookahead_point = nearest_point
            else:
                lookahead_point = None

            if not isinstance(lookahead_point, np.ndarray):
                dist = (current_pose[0] - self.trajectory.points[-1][0])**2 + (
                    current_pose[1] - self.trajectory.points[-1][1])**2
                if dist > self.max_reacquire**2:
                    err += best_err + 1
                # print "Cannot get the end, stopping Car"
            else:
                # print "move_point: ", current_pose
                steering_angle = self.determine_steering_angle(
                    current_pose, lookahead_point)

                if first_angle:
                    prev_angle = steering_angle
                    int_cte = 0.
                    first_angle = False

                diff_cte = steering_angle - prev_angle
                pid_angle = pid[0] * steering_angle + pid[2] * diff_cte + pid[1] * int_cte
                if np.abs(pid_angle) > self.max_angle:
                    pid_angle = self.max_angle * np.sign(pid_angle)
                # TODO: Wrong!
                # err += np.abs(pid_angle - prev_angle)
                err += nearest_dist + np.abs(prev_angle - pid_angle)

                # print "err", err, "; angle", pid_angle, "; current_pose", current_pose
                int_cte += pid_angle
                prev_angle = pid_angle

                current_pose = self.move(
                    current_pose, self.speed, pid_angle, duration=1.0)
                dist = (current_pose[0] - pre_pose[0])**2 + (
                    current_pose[1] - pre_pose[1])**2
                if dist < 0.001:
                    err += (self.lookahead + 1) / (dist + 0.1)
                if self.isCollision(current_pose, self.map_data, self.margin):
                    # print "collision!"
                    err += best_err + 1
                if self.viz:
                    tmp = Pose()
                    tmp.position.x = current_pose[0]
                    tmp.position.y = current_pose[1]
                    tmp.position.z = 0.
                    tmp.orientation = utils.angle_to_quaternion(
                        current_pose[2])
                    pa.poses.append(tmp)
                    # if step % 30 == 0:
                    #     pub.publish(pa)
        if self.viz and err <= best_err:
            print 'Better pid params found.'
            pub.publish(pa)
            time.sleep(0.1)
        # print "End measurement: err", err
        return err

    def move(self,
             pose,
             speed=0.,
             angle=0.,
             duration=1.,
             width=0.335,
             length=0.335):
        goal = [0., 0., 0.]
        if np.abs(angle) < 0.001:
            dist = speed * duration
            goal[0] = pose[0] + dist * np.cos(pose[2])
            goal[1] = pose[1] + dist * np.sin(pose[2])
            goal[2] = pose[2]
            return goal
        arc_len = speed * duration
        R = length / np.abs(np.tan(angle)) + width / 2
        beta = (arc_len / R) % (2 * np.pi) * np.sign(angle)
        np.random.seed(0)
        beta += np.random.normal(0, self.angle_noise)

        xp = pose[0] + R * np.sin(-pose[2]) * np.sign(angle)
        yp = pose[1] + R * np.cos(-pose[2]) * np.sign(angle)
        goal[0] = xp + (pose[0] - xp) * np.cos(beta) - (
            pose[1] - yp) * np.sin(beta)
        goal[1] = yp + (pose[0] - xp) * np.sin(beta) + (
            pose[1] - yp) * np.cos(beta)
        goal[2] = (pose[2] + beta) % (2 * np.pi)
        return goal

    def isCollision(self, point, map_data, radius=0.3):
        ''' Check the point is radius long away from the nearest obstacle in map_data or not
        '''
        squared_radius = int(radius / map_data.info.resolution)**2
        px = int((point[0] - map_data.info.origin.position.x) /
                 map_data.info.resolution)
        py = int((point[1] - map_data.info.origin.position.y) /
                 map_data.info.resolution)
        for x in xrange(
                int((point[0] - map_data.info.origin.position.x - radius) /
                    map_data.info.resolution),
                int((point[0] - map_data.info.origin.position.x + radius) /
                    map_data.info.resolution)):
            if x >= 0 and x < map_data.info.width:
                for y in xrange(
                        int((point[1] - map_data.info.origin.position.y -
                             radius) / map_data.info.resolution),
                        int((point[1] - map_data.info.origin.position.y +
                             radius) / map_data.info.resolution)):
                    if y >= 0 and y < map_data.info.height:
                        if (x - px)**2 + (
                                y - py
                        )**2 <= squared_radius and map_data.data[y * map_data.
                                                                 info.width +
                                                                 x] != 0:
                            return True
        return False


if __name__ == "__main__":
    rospy.init_node("pure_pursuit")
    print "Start pure pursuit..."
    pf = PurePursuit()
    rospy.spin()

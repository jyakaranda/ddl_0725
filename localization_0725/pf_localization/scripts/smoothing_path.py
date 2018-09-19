#!/usr/bin/env python
# -*- coding: UTF-8 -*-

from copy import deepcopy

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from utils import angle_to_quaternion, make_header


class Smoothing(object):
    def __init__(self):
        self.param_margin_ = rospy.get_param('~margin', default=0.3)
        self.param_alpha_ = rospy.get_param('~alpha', default=0.1)
        self.param_beta_ = rospy.get_param('~beta', default=0.3)
        self.param_iterations_ = rospy.get_param('~iterations', default=50)
        self.param_tolerance_ = rospy.get_param('~tolerance', default=0.5)
        self.param_min_point_dist_ = rospy.get_param(
            '~min_point_dist', default=0.5)
        self.param_min_point_dist_ = self.param_min_point_dist_**2
        self.param_map_frame_ = rospy.get_param('~map_frame', default='/map')

        self.map_data_ = None
        self.raw_path_ = None

        self.sub_map_ = rospy.Subscriber(
            '/map', OccupancyGrid, self.mapCB, queue_size=1)
        self.sub_raw_path_ = rospy.Subscriber(
            '/raw_path', Path, self.rawPathCB, queue_size=1)
        self.pub_smoothed_path_ = rospy.Publisher(
            '/smoothed_path', Path, queue_size=2)

        print 'init ok.'

    def mapCB(self, msg):
        ''' Save the static map info to map_data_
        '''
        if isinstance(self.map_data_, OccupancyGrid):
            return
        self.map_data_ = msg

    def rawPathCB(self, msg):
        ''' 1. Save raw path to raw_path_.
            2. Smooth path by gradient descending.
            3. Remove point from smoothed path which is too close to its neighborhood points ,or its neighborhood points is too close(which means there is probably a peak in path).
            4. Publish the result path
        '''
        self.raw_path_ = msg

        if not isinstance(self.map_data_, OccupancyGrid):
            print 'Received raw path, but cannot smooth when map data not received.'
            return

        diff = self.param_tolerance_ + 1
        step = 0
        np_path = self.makeNpArray(self.raw_path_)
        if not isinstance(np_path, object):
            return
        new_path = deepcopy(np_path)

        while step < self.param_iterations_:
            if diff < self.param_tolerance_:
                break

            step += 1
            diff = 0.
            pre_path = deepcopy(new_path)

            i = 1
            while i != new_path.shape[0] - 2:
                new_path[i] += self.param_alpha_ * (
                    pre_path[i] - new_path[i]) + self.param_beta_ * (
                        new_path[i - 1] + new_path[i + 1] - 2 * new_path[i])
                if self.isCollision(new_path[i], self.map_data_,
                                    self.param_margin_):
                    new_path[i] = deepcopy(pre_path[i])
                    i += 1
                    continue
                # if np.sum((new_path[i] - new_path[i - 1])**
                #           2) < self.param_min_point_dist_ or np.sum(
                #               (new_path[i] - new_path[i + 1])**
                #               2) < self.param_min_point_dist_ or np.sum(
                #                   (new_path[i - 1] - new_path[i + 1])**
                #                   2) < self.param_min_point_dist_:
                if np.sum((new_path[i - 1] - new_path[i + 1])**
                          2) < self.param_min_point_dist_:
                    new_path = np.delete(new_path, i, axis=0)
                    pre_path = np.delete(pre_path, i, axis=0)
                    i -= 1
                i += 1

            diff += np.sum((new_path - pre_path)**2)

        print 'round: ', step, '; diff: ', diff, '; origin # of points: ', len(
            self.raw_path_.poses
        ), '; result # of points: ', new_path.shape[
            0], '; # of deleted points: ', np_path.shape[0] - new_path.shape[0]

        smoothed_path = Path()
        smoothed_path.header = make_header(self.param_map_frame_)
        for i in xrange(new_path.shape[0]):
            pose = PoseStamped()
            pose.pose.position.x = new_path[i][0]
            pose.pose.position.y = new_path[i][1]
            pose.pose.position.z = 0
            pose.pose.orientation = angle_to_quaternion(0)
            smoothed_path.poses.append(pose)
        self.pub_smoothed_path_.publish(smoothed_path)

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

    def makeNpArray(self, path):
        if not isinstance(path, Path) or len(path.poses) <= 2:
            print 'Cannot make a np array when path is not instance of Path, Or number of poses is not more than 2'
            return

        np_path = np.zeros((len(path.poses), 2))
        for i in xrange(np_path.shape[0]):
            np_path[i][0] = path.poses[i].pose.position.x
            np_path[i][1] = path.poses[i].pose.position.y

        return np_path


if __name__ == '__main__':
    rospy.init_node('smoothing_path')
    print 'Start smoothing path...'
    sm = Smoothing()
    rospy.spin()

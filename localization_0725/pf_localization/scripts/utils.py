#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import collections
import json
import time

import numpy as np
import recordclass
import rospy
import skimage.morphology
import tf
import tf.transformations
from geometry_msgs.msg import (Point, Point32, PointStamped, Polygon,
                               PolygonStamped, Pose, PoseArray, PoseStamped,
                               PoseWithCovarianceStamped, Quaternion, Vector3)
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as navPath
from scipy import ndimage
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from yaml import load

EPSILON = 0.00000000001
''' These data structures are used in the search function
'''

Circle = collections.namedtuple("Circle",
                                ["radius", "center", "angle", "deflection"])
Path = collections.namedtuple("Path", ["states"])
# wraps a state with additional search information
SearchNode = collections.namedtuple("SearchNode",
                                    ["state", "cost", "heuristic", "parent"])
SearchNodeTree = collections.namedtuple(
    "SearchNodeTree", ["state", "cost", "heuristic", "parent", "tree_node"])
# used for recreating the search tree
TreeNode = recordclass.recordclass("TreeNode", ["state", "children"])


class Map(object):
    """ Convenience wrapper for an occupancy grid map object.
	    Provides methods to:
	        - check distances to nearest objects
	        - check if points are permissible
	        - mark map regions as explored
	        - dilate the map
	"""

    def __init__(self, map_msg):
        self.memoize_disks = True
        self.map_msg = map_msg
        self.map_info = map_msg.info
        #  # 0: permissible, -1: unmapped, 100: blocked
        self.raw_matrix = np.array(map_msg.data).reshape((map_msg.info.height,
                                                          map_msg.info.width))
        self.exploration_coeff = float(
            rospy.get_param("~exploration_coeff", 0.75))

        # reversed from expectation since this is what distance_transform_edt requires
        self.occupancy_grid = np.ones_like(self.raw_matrix, dtype=bool)
        self.occupancy_grid[self.raw_matrix > 50] = 0

        # 0: not permissible, 1: permissible
        self.permissible_region = np.zeros_like(self.raw_matrix, dtype=bool)
        self.permissible_region[self.raw_matrix == 0] = 1

        self.distmap = ndimage.distance_transform_edt(self.occupancy_grid)
        self.exploration_buffer = np.zeros_like(self.raw_matrix, dtype=bool)

        if self.memoize_disks:
            self.memo_table = {}
            self.memoized = 0
            self.unmemoized = 0

    def get_permissible(self, queries, check_bounds=False, coord_convert=True):
        ''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
        if coord_convert:
            q = queries.copy()
            world_to_map(q, self.map_info)
            q = np.round(q[:, :2]).astype(int)
        else:
            q = queries.astype(int)

        if check_bounds:
            bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
                  np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
                  np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
            q[bad, :] = 0

        distances = self.permissible_region[q[:, 1], q[:, 0]]
        if check_bounds:
            distances[bad] = np.nan
        return distances

    def get_distances(self, queries, check_bounds=False, coord_convert=True):
        ''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
        if coord_convert:
            q = queries.copy()
            world_to_map(q, self.map_info)
            q = np.round(q[:, :2]).astype(int)
        else:
            q = queries.astype(int)

        if check_bounds:
            bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
                  np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
                  np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
            q[bad, :] = 0

        distances = self.distmap[q[:, 1], q[:, 0]] * self.map_info.resolution
        if check_bounds:
            distances[bad] = np.nan
        return distances

    def clear_exploration_buffer(self):
        self.exploration_buffer.fill(0)

    def add_circle_to_exploration_buffer(self, circle):
        ''' marks a circular region of the exploration_buffer as explored
		'''
        position = np.array([circle.center.copy()])
        world_to_map(position, self.map_info)
        radius_pixels = int(
            np.ceil(circle.radius / self.map_info.resolution) * 0.75)
        x_center, y_center = int(position[0, 1]), int(position[0, 0])
        mask = skimage.morphology.disk(radius_pixels).astype(bool)
        half_size = int(mask.shape[0] / 2.0)
        self.exploration_buffer[x_center - half_size:x_center + half_size +
                                1, y_center - half_size:y_center + half_size +
                                1] += mask

    def add_circles_to_exploration_buffer(self, poses, radii, exp_coeff=None):
        if exp_coeff == None:
            exp_coeff = self.exploration_coeff
        world_to_map(poses, self.map_info)
        radii_pixels = (
            np.ceil(radii / self.map_info.resolution) * exp_coeff).astype(int)

        for i in xrange(poses.shape[0]):
            x_center, y_center = int(poses[i, 1]), int(poses[i, 0])
            if self.memoize_disks and radii_pixels[i] in self.memo_table:
                mask = self.memo_table[radii_pixels[i]]
                self.memoized += 1
            else:
                mask = skimage.morphology.disk(radii_pixels[i]).astype(bool)
                if self.memoize_disks:
                    self.unmemoized += 1
                    self.memo_table[radii_pixels[i]] = mask
            half_size = int(mask.shape[0] / 2.0)
            # print half_size
            self.exploration_buffer[x_center - half_size:x_center + half_size +
                                    1, y_center - half_size:y_center +
                                    half_size + 1] += mask

    def get_explored(self, queries, check_bounds=False, coord_convert=True):
        ''' Given a Nx3 (x,y,theta) numpy array of queries, this returns the distances to the nearest obstacle at each query position.
		'''
        if coord_convert:
            q = queries.copy()
            world_to_map(q, self.map_info)
            q = np.round(q[:, :2]).astype(int)
        else:
            q = queries.astype(int)

        if check_bounds:
            bad = np.unique(np.concatenate((np.argwhere(q<0)[:,0], \
                  np.argwhere(q[:,1] >= self.occupancy_grid.shape[0])[:,0],  \
                  np.argwhere(q[:,0] >= self.occupancy_grid.shape[1])[:,0])))
            q[bad, :] = 0

        explored = self.exploration_buffer[q[:, 1], q[:, 0]]
        if check_bounds:
            explored[bad] = np.nan
        return explored

    def get_exploration_occupancy_grid(self):
        msg = OccupancyGrid()
        msg.header = make_header("/map")
        msg.info = self.map_info
        buff = self.exploration_buffer.astype(np.int8)
        buff[buff == 0] = 100
        buff[buff == 1] = 50
        msg.data = buff.reshape(
            self.map_info.height * self.map_info.width).tolist()
        return msg

    def dilate(self, radius):
        og = np.zeros_like(self.occupancy_grid, dtype=float)
        og[self.occupancy_grid == 0] = 255
        el = skimage.morphology.disk(10)
        return skimage.morphology.dilation(og, selem=el)


def load_params_from_yaml(fp):
    with open(fp, 'r') as infile:
        yaml_data = load(infile)
        for param in yaml_data:
            print "param:", param, ":", yaml_data[param]
            rospy.set_param("~" + param, yaml_data[param])


class CircularArray(object):
    """docstring for CircularArray"""

    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    def mean(self):
        return np.mean(self.arr[:self.num_els])

    def median(self):
        return np.median(self.arr[:self.num_els])


class Timer:
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    def fps(self):
        return self.arr.mean()


def marker_clear_all(frame_id):
    # Create a marker which clears all.
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.action = 3  # DELETEALL action.
    return marker


def marker_from_circle(circle,
                       index=0,
                       linewidth=0.1,
                       color=ColorRGBA(1, 0, 0, 1),
                       z=0.,
                       lifetime=10.0):
    marker = Marker()
    marker.header = make_header("/map")

    marker.ns = "Markers_NS"
    marker.id = index
    marker.type = Marker.CYLINDER
    marker.action = 0  # action=0 add/modify object
    # marker.color.r = 1.0
    # marker.color.g = 0.0
    # marker.color.b = 0.0
    # marker.color.a = 0.4
    marker.color = color
    marker.lifetime = rospy.Duration.from_sec(lifetime)

    marker.pose = Pose()
    marker.pose.position.z = z
    marker.pose.position.x = circle.center[0]
    marker.pose.position.y = circle.center[1]

    marker.scale = Vector3(circle.radius * 2.0, circle.radius * 2.0, 0)
    return marker


def marker_from_point_radius(point,
                             radius,
                             index=0,
                             linewidth=0.1,
                             color=ColorRGBA(1, 0, 0, 1),
                             z=0.,
                             lifetime=10.0):
    marker = Marker()
    marker.header = make_header("/map")

    marker.ns = "Speed_NS"
    marker.id = index
    marker.type = Marker.CYLINDER
    marker.action = 0  # action=0 add/modify object
    # marker.color.r = 1.0
    # marker.color.g = 0.0
    # marker.color.b = 0.0
    # marker.color.a = 0.4
    marker.color = color
    marker.lifetime = rospy.Duration.from_sec(lifetime)

    marker.pose = Pose()
    marker.pose.position.z = z
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]

    marker.scale = Vector3(radius * 2.0, radius * 2.0, 0.001)
    return marker


def make_circle_marker(point,
                       scale,
                       color,
                       frame_id,
                       namespace,
                       sid,
                       duration=0):
    marker = Marker()
    marker.header = make_header(frame_id)
    marker.ns = namespace
    marker.id = sid
    marker.type = 2  # sphere
    marker.lifetime = rospy.Duration.from_sec(duration)
    marker.action = 0
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.orientation.w = 1.0
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = 1.0
    return marker


def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


def quaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
	The angle represents the yaw.
	This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw


def rotation_matrix(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.matrix([[c, -s], [s, c]])


def particle_to_pose(particle):
    pose = Pose()
    pose.position.x = particle[0]
    pose.position.y = particle[1]
    pose.orientation = angle_to_quaternion(particle[2])
    return pose


def particles_to_poses(particles):
    return map(particle_to_pose, particles)


def make_header(frame_id, stamp=None):
    if stamp == None:
        stamp = rospy.Time.now()
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def point(npt):
    pt = Point32()
    pt.x = npt[0]
    pt.y = npt[1]
    return pt


def points(arr):
    return map(point, arr)


# converts map space coordinates to world space coordinates
def map_to_world_slow(x, y, t, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(angle)
    trans = np.array([[map_info.origin.position.x],
                      [map_info.origin.position.y]])

    map_c = np.array([[x], [y]])
    world = (rot * map_c) * scale + trans

    return world[0, 0], world[1, 0], t + angle


def map_to_world(poses, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)

    # rotate

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]

    # scale
    poses[:, :2] *= float(scale)

    # translate
    poses[:, 0] += map_info.origin.position.x
    poses[:, 1] += map_info.origin.position.y
    poses[:, 2] += angle


def world_to_map(poses, map_info):
    # equivalent to map_to_grid(world_to_map(poses))
    # operates in place
    scale = map_info.resolution
    angle = -quaternion_to_angle(map_info.origin.orientation)

    # translation
    poses[:, 0] -= map_info.origin.position.x
    poses[:, 1] -= map_info.origin.position.y

    # scale
    poses[:, :2] *= (1.0 / float(scale))

    # rotation
    c, s = np.cos(angle), np.sin(angle)
    # we need to store the x coordinates since they will be overwritten
    temp = np.copy(poses[:, 0])
    poses[:, 0] = c * poses[:, 0] - s * poses[:, 1]
    poses[:, 1] = s * temp + c * poses[:, 1]
    if poses.shape[1] > 2:
        poses[:, 2] += angle


# converts world space coordinates to map space coordinates
def world_to_map_slow(x, y, t, map_info):
    scale = map_info.resolution
    angle = quaternion_to_angle(map_info.origin.orientation)
    rot = rotation_matrix(-angle)
    trans = np.array([[map_info.origin.position.x],
                      [map_info.origin.position.y]])

    world = np.array([[x], [y]])
    map_c = rot * ((world - trans) / float(scale))
    return map_c[0, 0], map_c[1, 0], t - angle


# finds the nearest point on the given line segment connecting start and end
# all arguments should be numpy arrays
def nearest_point_on_line_segment(point, start, end):
    '''
	Return the nearest point along the line segment connecting start and end.

	>>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,1.0]), np.array([1.0,-1.0]))
	(array([ 1.,  0.]), 0.5)

	>>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,1.0]), np.array([1.0,2.0]))
	(array([ 1.,  1.]), 0.0)

	>>> nearest_point_on_line_segment(np.array([0.0,0.0]), np.array([1.0,-2.0]), np.array([1.0,-1.0]))
	(array([ 1., -1.]), 1.0)
	'''
    diff = start - end
    l2 = np.dot(diff, diff)
    if l2 == 0.0:
        return start, 0.0
    t = np.clip(np.dot(point - start, end - start) / l2, 0.0, 1.0)
    projection = start + t * (end - start)
    return projection, t


def nearest_point_on_trajectory(point, trajectory):
    '''
	Return the nearest point along the given piecewise linear trajectory.

	Same as nearest_point_on_line_segment, but vectorized. This method is quite fast, time constraints should
	not be an issue so long as trajectories are not insanely long. 

		Order of magnitude: trajectory length: 1000 --> 0.0002 second computation (5000fps)

	point: size 2 numpy array
	trajectory: Nx2 matrix of (x,y) trajectory waypoints
		- these must be unique. If they are not unique, a divide by 0 error will destroy the world
	'''
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0]**2 + diffs[:, 1]**2
    # this is equivalent to the elementwise dot product
    dots = np.sum((point - trajectory[:-1, :]) * diffs[:, :], axis=1)
    t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1, :] + (t * diffs.T).T
    dists = np.linalg.norm(point - projections, axis=1)
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[
        min_dist_segment], min_dist_segment


def first_point_on_trajectory_intersecting_circle(point,
                                                  radius,
                                                  trajectory,
                                                  t=0.0,
                                                  wrap=False):
    ''' starts at beginning of trajectory, and find the first point one radius away from the given point along the trajectory.

	Assumes that the first segment passes within a single radius of the point

	http://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
	'''
    start_i = int(t)
    start_t = t % 1.0
    first_t = None
    first_i = None
    first_p = None
    for i in xrange(start_i, trajectory.shape[0] - 1):
        start = trajectory[i, :]
        end = trajectory[i + 1, :]
        V = end - start

        a = np.dot(V, V)
        b = 2.0 * np.dot(V, start - point)
        c = np.dot(start, start) + np.dot(
            point, point) - 2.0 * np.dot(start, point) - radius * radius
        discriminant = b * b - 4 * a * c

        if discriminant < 0:
            continue
        # 	print "NO INTERSECTION"
        # else:
        # if discriminant >= 0.0:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2.0 * a)
        t2 = (-b + discriminant) / (2.0 * a)
        if i == start_i:
            if t1 >= 0.0 and t1 <= 1.0 and t1 >= start_t:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            if t2 >= 0.0 and t2 <= 1.0 and t2 >= start_t:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break
        elif t1 >= 0.0 and t1 <= 1.0:
            first_t = t1
            first_i = i
            first_p = start + t1 * V
            break
        elif t2 >= 0.0 and t2 <= 1.0:
            first_t = t2
            first_i = i
            first_p = start + t2 * V
            break
    # wrap around to the beginning of the trajectory if no intersection is found1
    if wrap and first_p == None:
        for i in xrange(start_i):
            start = trajectory[i, :]
            end = trajectory[i + 1, :]
            V = end - start

            a = np.dot(V, V)
            b = 2.0 * np.dot(V, start - point)
            c = np.dot(start, start) + np.dot(
                point, point) - 2.0 * np.dot(start, point) - radius * radius
            discriminant = b * b - 4 * a * c

            if discriminant < 0:
                continue
            discriminant = np.sqrt(discriminant)
            t1 = (-b - discriminant) / (2.0 * a)
            t2 = (-b + discriminant) / (2.0 * a)
            if t1 >= 0.0 and t1 <= 1.0:
                first_t = t1
                first_i = i
                first_p = start + t1 * V
                break
            elif t2 >= 0.0 and t2 <= 1.0:
                first_t = t2
                first_i = i
                first_p = start + t2 * V
                break

    return first_p, first_i, first_t

    # print min_dist_segment, dists[min_dist_segment], projections[min_dist_segment]


# coords: Nx2 in polar (r,theta)
# in place modifies to Nx2 (x,y)
def polar_to_euclid(coords):
    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return (xs, ys)


def angular_deflection_magnitude(points):
    # https://mail.python.org/pipermail/tutor/2007-July/055178.html
    # returns a numpy array of angular deflections between consequtive
    # line segments beginning and ending at the points provided
    # contains two less angles than points, since the angular deflection for the first and last components is ill defined

    lines = np.zeros((points.shape[0] - 1, 3))
    thetas = np.zeros(points.shape[0] - 2)
    for i in xrange(1, points.shape[0]):
        p0 = points[i - 1, :]
        p1 = points[i, :]

        A = p0[1] - p1[1]
        B = p1[0] - p0[0]
        C = p0[0] * p1[1] - p1[0] * p0[1]
        lines[i - 1] = (A, B, C)

    for i in xrange(1, lines.shape[0]):
        A1 = lines[i - 1, 0]
        B1 = lines[i - 1, 1]
        A2 = lines[i, 0]
        B2 = lines[i, 1]
        bottom = (A1**2 + B1**2) * (A2**2 + B2**2)
        if bottom > 0:
            inner = (A1 * A2 + B1 * B2) / np.sqrt(bottom)
            # thetas[i-1] = np.arccos(inner)
            if np.abs(np.abs(inner) - 1.0) < EPSILON:
                thetas[i - 1] = 0.0
            else:
                thetas[i - 1] = np.arccos(inner)
    return thetas


def piecewise_linear_local_waypoints_polar(points):
    thetas = angular_deflection_magnitude(points)
    # # compute the polar coordinate space local coordinate frame waypoints (r,theta)
    local_points_polar = np.zeros((points.shape[0] - 1, 2))
    for i in xrange(1, points.shape[0] - 1):
        # radius
        local_points_polar[i - 1, 0] = np.linalg.norm(points[i, :] -
                                                      points[i - 1, :])
        # angle
        local_points_polar[i, 1] = thetas[i - 1]
    local_points_polar[-1, 0] = np.linalg.norm(points[-1, :] - points[-2, :])
    return local_points_polar

    # local_points_cartesian = np.zeros_like(local_points_polar)
    # local_points_cartesian[:,0] = local_points_polar[:,0] * np.cos(local_points_polar[:,1])
    # local_points_cartesian[:,1] = local_points_polar[:,0] * np.sin(local_points_polar[:,1])
    # print local_points_cartesian
    # print local_points_polar


class AckermannModel(object):
    """ A wrapper class for useful Ackermann steering geometry related functions
	"""

    def __init__(self, wheelbase):
        self.L = wheelbase

    def path_radius(self, steering_angle):
        ''' The radius of the path driven if a constant steering angle is applied
		'''
        return self.L / np.tan(steering_angle)

    def yaw_rate(self, steering_angle, speed):
        ''' Rate of change of heading with a given steering angle and speed
		'''
        if steering_angle == 0.0:
            return 0.0
        return speed / self.path_radius(steering_angle)

    def dx(self, speed, dt, steering_angle):
        ''' Distance traveled in the local x direction given speed and steering_angle
		'''
        if steering_angle == 0.0:
            return speed * dt
        R = self.path_radius(steering_angle)
        d = dt * speed
        dx = R * np.sin(d / R)
        return dx

    def dy(self, speed, dt, steering_angle):
        ''' Distance traveled in the local y direction given speed and steering_angle
		'''
        if steering_angle == 0.0:
            return 0.0
        R = self.path_radius(steering_angle)
        d = dt * speed
        dy = R * (1.0 - np.cos(d / R))
        return dy

    def steering_angle(self, point):
        ''' Returns the steering angle required to pass through the given point
		    (in local euclidean coordinates) assuming constant steering angle is applied
		'''
        if point[0] >= 0.0:
            theta = np.arctan(point[1] / point[0])
        else:
            theta = np.arctan(
                abs(point[0]) / point[1]) + np.sign(point[1]) * np.pi / 2.0

        return np.arctan(2.0 * self.L * np.sin(theta) / np.linalg.norm(point))

    def steering_angle_polar(self, polar_point):
        ''' Returns the steering angle required to pass through the given point
		    (in local polar coordinates) assuming constant steering angle is applied
		'''
        theta = polar_point[1]
        radius = polar_point[0]
        return np.arctan(2.0 * self.L * np.sin(theta) / radius)


def max_angle(min_turning_radius, radius):
    tr2 = 2.0 * min_turning_radius
    if radius < tr2:
        r2 = radius * radius
        y = r2 / (2.0 * min_turning_radius)
        x = np.sqrt(r2 - y * y)
        max_angle = np.arctan(y / x)
    else:
        max_angle = np.pi / 2.0
    return max_angle


class LineTrajectory(object):
    """ A class to wrap and work with piecewise linear trajectories. """

    def __init__(self, viz_namespace=None):
        self.points = []
        self.np_points = None
        self.distances = []
        self.speed_profile = []
        self.has_acceleration = False
        self.visualize = False
        self.viz_namespace = viz_namespace
        self.speed_interpolater = None

        if viz_namespace:
            self.visualize = True
            self.start_pub = rospy.Publisher(
                viz_namespace + "/start_point", Marker, queue_size=1)
            self.traj_pub = rospy.Publisher(
                viz_namespace + "/path", Marker, queue_size=1)
            self.end_pub = rospy.Publisher(
                viz_namespace + "/end_pose", Marker, queue_size=1)
            self.speed_pub = rospy.Publisher(
                viz_namespace + "/speeds", MarkerArray, queue_size=1)
            self.pub_path = rospy.Publisher(
                viz_namespace + "/recorded_path", navPath, queue_size=1)

    # compute the distances along the path for all path segments beyond those already computed
    def update_distances(self):
        num_distances = len(self.distances)
        num_points = len(self.points)

        for i in xrange(num_distances, num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i - 1]
                p1 = self.points[i]
                delta = np.array([p0[0] - p1[0], p0[1] - p1[1]])
                self.distances.append(self.distances[i - 1] +
                                      np.linalg.norm(delta))

    def distance_to_end(self, t):
        if not len(self.points) == len(self.distances):
            print "WARNING: Different number of distances and points, this should never happen! Expect incorrect results. See LineTrajectory class."
        dat = self.distance_along_trajectory(t)
        if dat == None:
            return None
        else:
            return self.distances[-1] - dat

    # 路径 index 为 t 的点到终点的距离，这里的 t 可能是小数，那就线性插值进行计算
    def distance_along_trajectory(self, t):
        # compute distance along path
        # ensure path boundaries are respected
        if t < 0 or t > len(self.points) - 1.0:
            return None
        i = int(t)  # which segment
        t = t % 1.0  # how far along segment
        if t < EPSILON:
            return self.distances[i]
        else:
            # return self.distances[i] + t*(self.distances[i+1]-self.distances[i]) 插值
            return (1.0 - t) * self.distances[i] + t * self.distances[i + 1]

    def addPoint(self, x, y, theta=0):
        print "adding point to trajectory:", x, y
        self.points.append((x, y, theta))
        self.update_distances()
        self.mark_dirty()

    def clear(self):
        self.points = []
        self.distances = []
        self.speed_profile = []
        self.speed_interpolater = None
        self.mark_dirty()

    def empty(self):
        return len(self.points) == 0

    def save(self, path):
        print "Saving trajectory to:", path
        data = {}
        data["points"] = []
        for p in self.points:
            data["points"].append({"x": p[0], "y": p[1], "theta": p[2]})
        with open(path, 'w') as outfile:
            json.dump(data, outfile)

    def mark_dirty(self):
        self.has_acceleration = False

    def dirty(self):
        return not self.has_acceleration

    def load(self, path):
        print "Loading trajectory:", path
        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                self.points.append((p["x"], p["y"], p["theta"]))
        self.update_distances()
        print "Loaded:", len(self.points), "points"
        self.mark_dirty()

    # put the points into a KD tree for faster nearest neighbors queries
    def make_np_array(self):
        self.np_points = np.array(self.points)
        self.np_distances = np.array(self.distances)
        self.has_acceleration = True

    # build a trajectory class instance from a trajectory message
    def fromPolygon(self, trajMsg):
        for p in trajMsg.points:
            self.points.append((p.x, p.y))
            if p.z >= 0:
                self.speed_profile.append(p.z)
        self.update_distances()
        self.mark_dirty()
        print "Loaded new trajectory with:", len(self.points), "points"

    def toPolygon(self):
        poly = PolygonStamped()
        poly.header = make_header("/map")
        use_speed_profile = len(self.speed_profile) == len(self.points)
        for i in xrange(len(self.points)):
            p = self.points[i]
            pt = Point32()
            pt.x = p[0]
            pt.y = p[1]
            if use_speed_profile:
                pt.z = self.speed_profile[i]
            else:
                pt.z = -1
            poly.polygon.points.append(pt)
        return poly

    def fromPath(self, path):
        for p in path.poses:
            self.points.append((p.pose.position.x, p.pose.position.y))
            if p.pose.position.z > 0:
                self.speed_profile.append(p.pose.position.z)
            self.update_distances()
            self.mark_dirty()
            print "Loaded new trajectory with:", len(path.poses),"points."


    def toPath(self):
        path = navPath()
        path.header = make_header("/map")
        for p in self.points:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = 0
            pose.pose.orientation = angle_to_quaternion(p[2])
            path.poses.append(pose)
        return path

    def publish_start_point(self, duration=0.0, scale=0.1):
        should_publish = len(self.points) > 0
        if self.visualize and self.speed_pub.get_num_connections() > 0:
            print "Publishing speed profile"
            marker = Marker()
            marker.header = make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 0
            marker.type = 2  # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[0][0]
                marker.pose.position.y = self.points[0][1]
                marker.pose.orientation.w = 1
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.start_pub.publish(marker)
        # elif self.start_pub.get_num_connections() == 0:
        # 	print "Not publishing start point, no subscribers"

    def publish_end_point(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.end_pub.get_num_connections() > 0:
            print "Publishing end point"
            marker = Marker()
            marker.header = make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 1
            marker.type = 2  # sphere
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[-1][0]
                marker.pose.position.y = self.points[-1][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.end_pub.publish(marker)
        # elif self.end_pub.get_num_connections() == 0:
        # 	print "Not publishing end point, no subscribers"

    def publish_trajectory(self, duration=0.0):
        should_publish = len(self.points) > 1
        if self.visualize and self.traj_pub.get_num_connections() > 0:
            print "Publishing trajectory"
            marker = Marker()
            marker.header = make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 2
            marker.type = 4  # line strip
            marker.lifetime = rospy.Duration.from_sec(duration)
            if should_publish:
                marker.action = 0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 0.5
                for p in self.points:
                    pt = Point32()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = -0.1
                    marker.points.append(pt)
            else:
                # delete
                marker.action = 2
            self.traj_pub.publish(marker)
        elif self.traj_pub.get_num_connections() == 0:
            print "Not publishing trajectory, no subscribers"

    def publish_speeds(self, duration=0.0, scale=0.7):
        should_publish = len(self.speed_profile) > 1
        if self.visualize and self.speed_pub.get_num_connections() > 0:
            if self.dirty():
                self.make_np_array()
            markers = [marker_clear_all("/map")]
            normed_speeds = np.array(self.speed_profile) / np.max(
                self.speed_profile)
            last_speed = 0.0
            for i, speed in enumerate(normed_speeds):
                if speed >= last_speed * 0.99:
                    color = ColorRGBA(0, 1, 0, 0.8)
                else:
                    color = ColorRGBA(1, 0, 0, 0.8)
                last_speed = speed
                markers.append(
                    marker_from_point_radius(
                        self.np_points[i, :],
                        np.power(speed, 0.8) * scale,
                        index=i,
                        linewidth=0.05,
                        color=color,
                        lifetime=duration))

            marker_array = MarkerArray(markers=markers)
            self.speed_pub.publish(marker_array)

    def speed_at_t(self, t):
        if self.speed_interpolater == None:
            self.xs = np.arange(len(self.speed_profile))
            self.ys = self.speed_profile
            self.speed_interpolater = lambda x: np.interp(x, self.xs, self.ys)

        return self.speed_interpolater(t)

    def publish_viz(self, duration=0):
        if not self.visualize:
            print "Cannot visualize path, not initialized with visualization enabled"
            return

        self.publish_start_point(duration=duration)
        self.publish_trajectory(duration=duration)
        self.publish_end_point(duration=duration)
        self.publish_speeds(duration=duration)

    def publish_path(self, duration=0):
        if not self.visualize:
            print "Cannot visualize path, not initialized with visualization enabled"
            return
        if pub_path.get_num_connections() > 0:
            path = navPath()
            path.header = make_header("/base_link")
            for p in self.points:
                pose = PoseStamped()
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.position.z = 0
                pose.pose.orientation = angle_to_quaternion(p[2])

                path.poses.append(pose)
            self.pub_path.publish(path)
        elif pub_path.get_num_connections() == 0:
            print "Not publishing recorded path, no subscriber."


def test_line_trajectory():
    l = LineTrajectory()
    l.addPoint(Point32(x=0, y=0))
    l.addPoint(Point32(x=0, y=1))
    l.addPoint(Point32(x=0, y=2))
    l.addPoint(Point32(x=0, y=3))
    l.addPoint(Point32(x=3, y=3))
    l.update_distances()

    print l.distances

    print l.distance_along_trajectory(-1.0), l.distance_to_end(-1.0)
    print l.distance_along_trajectory(0.0), l.distance_to_end(0.0)
    print l.distance_along_trajectory(1), l.distance_to_end(1)
    print l.distance_along_trajectory(1.3), l.distance_to_end(1.3)
    print l.distance_along_trajectory(2.5), l.distance_to_end(2.5)
    print l.distance_along_trajectory(3.0), l.distance_to_end(3.0)
    print l.distance_along_trajectory(4.0), l.distance_to_end(4.0)
    print l.distance_along_trajectory(4.1), l.distance_to_end(4.1)


def test_max_angle():
    print max_angle(1.0, 2.0)
    print max_angle(1.0, 1.999)
    print max_angle(1.0, 1.5)
    print max_angle(1.0, 1.0)


def test_nearest():
    trajectory = np.zeros((5, 2))

    # trajectory[0,:] = (1,-2)
    # trajectory[1,:] = (1,-1)
    # trajectory[2,:] = (1,1)
    # trajectory[3,:] = (1,3)
    # trajectory[4,:] = (1,6)
    # s = time.time()
    # print nearest_point_on_trajectory(np.array([0.0,0.0]), trajectory)
    # print "Done in:", time.time() - s, " seconds"

    trajectory[0, :] = (-3.0, 0.0)
    trajectory[1, :] = (3.0, 0.0)
    trajectory[2, :] = (5.0, 0.0)
    trajectory[3, :] = (6.0, 0.0)
    trajectory[4, :] = (7.0, 0.0)
    print first_point_on_trajectory_intersecting_circle(
        np.array([0.0, 0.0]), 2.0, trajectory, 0.5)


def test_angular_deflection_magnitude():
    points = np.zeros((5, 2))
    points[0, :] = (0, 0)
    points[1, :] = (0, 1)
    points[2, :] = (1, 1)
    points[3, :] = (1, 0)
    points[4, :] = (0, 0)
    # the angles of a box are all 90
    print angular_deflection(points) == np.ones(3) * np.pi / 2.0

    points = np.zeros((5, 2))
    points[0, :] = (0, 0)
    points[1, :] = (0, 1)
    points[2, :] = (-1, 1)
    points[3, :] = (-1, 0)
    points[4, :] = (0, 0)
    # the angles of a box are all 90
    print angular_deflection(points) == np.ones(3) * np.pi / 2.0


def test_piecewise_linear_local_waypoints_polar():
    # https://mai{l.python.org/pipermail/tutor/2007-July/055178.html
    points = np.zeros((5, 2))
    points[0, :] = (0, 0)
    points[1, :] = (0, 1)
    points[2, :] = (1, 1)
    points[3, :] = (1, 0)
    points[4, :] = (0, -0.1)
    # points[4,:] = (0, 0.0)

    model = AckermannModel(0.25)
    local_points_polar = piecewise_linear_local_waypoints_polar(points)

    for p in local_points_polar:
        print
        print p
        print model.steering_angle_polar(p)

    print
    # thetas = angular_deflection_magnitude(points)

    # print thetas

    # # # compute the polar coordinate space local coordinate frame waypoints (r,theta)
    # local_points_polar = np.zeros((points.shape[0]-1, 2))
    # for i in xrange(1, points.shape[0]-1):
    # 	# radius
    # 	local_points_polar[i-1,0] = np.linalg.norm(points[i,:] - points[i-1,:])

    # 	# angle
    # 	local_points_polar[i,1] = thetas[i-1]

    # local_points_polar[-1,0] = np.linalg.norm(points[-1,:] - points[-2,:])

    # print local_points_polar


# test_piecewise_linear_local_waypoints_polar()
# test_nearest()
# test_angular_deflection()
# test_max_angle()
# test_line_trajectory()

#!/usr/bin/env python

import collections
import heapq
import itertools
import math
import os
import threading
import time
from itertools import count

import matplotlib.pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as navPath
from nav_msgs.srv import GetMap
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import utils
from utils import Circle, Path, SearchNode, SearchNodeTree, TreeNode


class HeuristicSearch(object):
    """ Abstract class for heuristic search on the provided set of cost/admissibility/heuristic/neighbor functions """

    def __init__(self, make_tree=False):
        self.make_tree = make_tree

    def reset(self, start_state):
        # Frontier is a priority queue.
        self.closed_set = []
        self.frontier = []
        self.found_paths = []
        self.goal_state = self.goal()
        self.step_count = 0
        if self.make_tree:
            self.tree_root = TreeNode(state=start_state, children=[])
            ss = SearchNodeTree(state=start_state, parent=None, cost=0, tree_node=self.tree_root, \
               heuristic=self.heuristic(start_state, self.goal_state))
        else:
            ss = SearchNode(state=start_state, parent=None, cost=0, \
               heuristic=self.heuristic(start_state, self.goal_state))

        self.neighbor_count = 0

        # a global
        self.tiebreaker = count()
        heapq.heappush(self.frontier,
                       (ss.heuristic, next(self.tiebreaker), ss))

    def search(self, time_limit):
        if self.goal_state == None:
            return None

        start_time = time.time()
        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit:
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step()

    def _step(self, make_tree=False, neighbor_callback=None):
        """ Perform one iteration of heuristic search - extend a single node.
			Requires that self.frontier is not empty.
		"""
        # Pop the best path to split.
        parent_score, parent_count, parent_state = heapq.heappop(self.frontier)
        if self.make_tree:
            parent_tree_node = parent_state.tree_node
        # print "Expand node:", parent_state

        if neighbor_callback:
            neighbor_states = []

        for neighbor_state in self.neighbors(parent_state.state):
            # prune any path segments that are bound to fail
            if not self.should_bail(
                    neighbor_state,
                    self.goal_state) and self.is_admissible(neighbor_state):
                # print "   neighbor: ", neighbor_state
                # build the tree representation of the search
                if self.make_tree:
                    ntn = TreeNode(state=neighbor_state, children=[])
                    parent_tree_node.children.append(ntn)

                    nss = SearchNodeTree(state=neighbor_state, parent=parent_state, tree_node=ntn, \
                     cost=parent_state.cost + self.cost(neighbor_state),  \
                     heuristic=self.heuristic(neighbor_state, self.goal_state))
                else:
                    nss = SearchNode(state=neighbor_state, parent=parent_state, \
                     cost=parent_state.cost + self.cost(neighbor_state),  \
                     heuristic=self.heuristic(neighbor_state, self.goal_state))

                if neighbor_callback:
                    neighbor_states.append(neighbor_state)

                self.neighbor_count += 1

                score = nss.cost + nss.heuristic
                if self.goal_met(neighbor_state, self.goal_state):
                    # print "Met goal:", neighbor_state
                    heapq.heappush(self.found_paths,
                                   (score, next(self.tiebreaker), nss))
                else:
                    # print (score, len(self.frontier))
                    heapq.heappush(self.frontier,
                                   (score, next(self.tiebreaker), nss))

        if neighbor_callback:
            neighbor_callback(neighbor_states)

        # self.map.add_circle_to_exploration_buffer(parent_state.state)
        self.closed_set.append(parent_state.state)
        self.step_count += 1

    def best(self):
        if len(self.found_paths) > 0:
            return self.make_path(self.found_paths[0][2], True, False)
        return None

    def make_path(self, end_node, add_goal=False, remove_start=True):
        # a path is a list of control states, in the order of traversal
        # builds it in reverse order by following parent pointers, then reverses result
        path = [end_node.state]
        while end_node.parent:
            path.append(end_node.parent.state)
            end_node = end_node.parent
        # don't execute the start state
        if remove_start:
            path.pop()
        path.reverse()

        if add_goal:
            path.append(self.goal_state)
        return Path(states=path)

    """ The following functions should be overridden by the parent class
	"""

    def cost(self, path):
        # estimate the cost of traversing a given path
        raise NotImplementedError("HeuristicSearch cost not specified")

    def heuristic(self, state, goal_state):
        # return an estimate for cost to go between the given state and the goal state
        raise NotImplementedError("HeuristicSearch heuristic not specified")

    def goal(self):
        # return the goal state
        raise NotImplementedError("HeuristicSearch goal not specified")

    def goal_met(self):
        # return true if the goal state is met
        raise NotImplementedError("HeuristicSearch goal not specified")

    def should_bail(self, state, goal_state):
        # return true if the node should be terminated
        return False

    def is_admissible(self, state):
        # return true if the given state is considered admissible - fulfills hard constraints
        raise NotImplementedError(
            "HeuristicSearch is_admissible not specified")

    def neighbors(self, state):
        # return a discretized set of neighbors of the given state
        raise NotImplementedError("HeuristicSearch neighbors not specified")


class SpaceExploration(HeuristicSearch):
    """ This class implements lower dimensional search to provide a high quality
		heuristic for higher dimensional path planning. If this search fails to 
		provide a solution, then there is a good chance that no solution exists
		and the car should just stop

		# TODO:
			put expanded nodes into a KD tree for fast nearest neighbors search. 
				This can prevent getting stuck in dead ends

		See: 
		    http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=7353741
		    http://www6.in.tum.de/Main/Publications/ChenChao2013a.pdf
	"""

    def __init__(self, omap):
        # fetch relevant parameters
        self.branch_factor = int(rospy.get_param("~branch_factor"))
        self.min_turn_radius = float(rospy.get_param("~minimum_turn_radius"))
        self.soft_min_radius = float(rospy.get_param("~soft_min_radius"))
        self.soft_min_penalty = float(rospy.get_param("~soft_min_penalty"))
        self.hard_min_radius = float(rospy.get_param("~hard_min_radius"))
        self.heuristic_bias = float(rospy.get_param("~heuristic_bias"))
        self.min_goal_overlap = float(rospy.get_param("~min_goal_overlap"))
        # self.half_space_theta = float(rospy.get_param("~half_space_theta"))

        # cache reused values
        self.map = omap
        self.thetas = np.zeros(self.branch_factor, dtype=float)
        self.radii = np.zeros(self.branch_factor)
        self.euclidean_neighbors = np.zeros(
            (self.branch_factor, 2), dtype=np.float64)
        self.next_goal = None

        # initialize super
        super(SpaceExploration, self).__init__(make_tree=False)

    def add_states_to_exploration_buffer(self, neighbor_circles):
        if len(neighbor_circles):
            poses = np.array(map(lambda c: c.center, neighbor_circles))
            radii = np.array(map(lambda c: c.radius, neighbor_circles))
            self.map.add_circles_to_exploration_buffer(poses, radii)

    def reset(self, pose):
        # Frontier is a priority queue.
        start_state = Circle(
            center=pose[:2],
            radius=self.map.get_distances(np.array([pose]))[0],
            angle=pose[2],
            deflection=0)
        self.map.clear_exploration_buffer()
        super(SpaceExploration, self).reset(start_state)

    def cost(self, state):
        if state.radius < self.soft_min_radius:
            return state.radius * self.soft_min_penalty
        return state.radius

    def heuristic(self, state, goal_state):
        if goal_state == None:
            return np.inf
        return (np.linalg.norm(state.center - goal_state.center) -
                state.radius) * self.heuristic_bias

    def goal(self):
        return self.next_goal

    def set_goal(self, pose):
        r = self.map.get_distances(np.array([pose]))[0]
        self.next_goal = Circle(
            center=pose[:2], radius=r, angle=pose[2], deflection=0)

    def overlap(self, s1, s2, percentage=.15):
        # TODO/NOTE: this is a bit of a hack to go fast, percentage overlap not accurate
        if np.linalg.norm(s1.center - s2.center) > (s1.radius + s2.radius) * (
                1.0 - percentage):
            return False
        else:
            return True

    def goal_met(self, state, goal_state):
        return self.overlap(state, goal_state, self.min_goal_overlap)

    def is_admissible(self, state):
        return state.radius > self.hard_min_radius

    def search(self, time_limit):
        # same as before, but this returns early if paths are found
        start_time = time.time()

        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit and not len(
                self.found_paths):
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step(neighbor_callback=self.add_states_to_exploration_buffer)

    def neighbors(self, state):
        max_angle = utils.max_angle(self.min_turn_radius, state.radius)
        self.thetas = np.linspace(
            -max_angle, max_angle, num=self.branch_factor)
        self.euclidean_neighbors[:, 0] = state.radius * np.cos(
            self.thetas + state.angle) + state.center[0]
        self.euclidean_neighbors[:, 1] = state.radius * np.sin(
            self.thetas + state.angle) + state.center[1]

        # we use this several times, so compute this icky math here to avoid unnecessary computation
        euc = self.euclidean_neighbors.copy()
        utils.world_to_map(euc, self.map.map_info)
        euc = np.round(euc).astype(int)
        radii = self.map.get_distances(
            euc, coord_convert=False, check_bounds=True)
        mask = np.logical_and(
            np.invert(self.map.get_explored(euc, coord_convert=False)),
            self.map.get_permissible(euc, coord_convert=False))

        neighbors = map(lambda c,r,t: Circle(center=c.copy(), radius=r, angle=state.angle + t, deflection=t),
         self.euclidean_neighbors[mask,:], radii[mask], self.thetas[mask])
        return neighbors


class PathPlanner(HeuristicSearch):
    ''' This class is used to refine an initial trajectory from the space
	    exploration search. It uses the original path as a heuristic and 
	    integrates a term in the cost function which penalizes excessive
	    turning. Generates suboptimal but visually smooth paths.
	'''

    def __init__(self, omap):
        # fetch relevant parameters
        self.branch_factor = int(rospy.get_param("~fp_branch_factor"))
        self.min_turn_radius = float(
            rospy.get_param("~fp_minimum_turn_radius"))
        self.soft_min_radius = float(rospy.get_param("~soft_min_radius"))
        self.soft_min_penalty = float(rospy.get_param("~fp_soft_min_penalty"))
        self.hard_min_radius = float(rospy.get_param("~hard_min_radius"))
        self.heuristic_bias = float(rospy.get_param("~fp_heuristic_bias"))
        self.min_goal_overlap = float(rospy.get_param("~min_goal_overlap"))
        # self.half_space_theta = float(rospy.get_param("~half_space_theta"))
        self.exploration_coeff = float(
            rospy.get_param("~fp_exploration_coeff"))
        self.max_circle_radius = float(
            rospy.get_param("~fp_max_circle_radius"))

        self.map = omap
        self.next_goal = None
        self.rough_trajectory = None

        # initialize super
        super(PathPlanner, self).__init__(make_tree=False)

    def set_heursitic_trajectory(self, trajectory):
        self.rough_trajectory = trajectory

    def reset(self, pose):
        start_state = Circle(
            center=pose[:2],
            radius=self.map.get_distances(np.array([pose]))[0],
            angle=pose[2],
            deflection=0)
        self.map.clear_exploration_buffer()
        super(PathPlanner, self).reset(start_state)

    def cost(self, state):
        ''' This is a fairly ad hoc cost function which tries to minimize path length and
		    curvy paths.
		'''
        other_coeff = 0.0
        if state.radius < self.soft_min_radius:
            other_coeff += 0.5
            # print "softmax"
            # return state.radius * self.soft_min_penalty + 10.0*np.power(abs(state.deflection),10.0)
        # return state.radius
        # return np.power(state.radius, 0.75) + 1.0*np.power(abs(state.deflection),3.0)

        # if state.deflection == 0.0:
        # 	other_coeff = -0.15
        # return state.radius + 5.0*np.power(abs(state.deflection),2.0) + other_coeff

        # if state.deflection == 0.0:
        # 	other_coeff = -0.08
        # return state.radius + 3.0*np.power(abs(state.deflection),1.2) + other_coeff

        # if state.deflection == 0.0:
        # 	other_coeff = -0.02
        return state.radius + 1.5 * np.power(abs(state.deflection),
                                             1.) + other_coeff

        # if state.deflection == 0.0:
        # 	other_coeff = -0.1
        # return state.radius + 3.0*np.power(abs(state.deflection),1.) + other_coeff
        # return state.radius + 1.8*np.power(abs(state.deflection),1.0) + other_coeff

    def heuristic(self, state, goal_state):
        ''' Finds the nearest point along the rough trajectory and then uses path length from there to the end as heursitic
		'''
        nearest_point, nearest_dist, t, i = utils.nearest_point_on_trajectory(
            state.center, self.rough_trajectory.np_points)
        distance_to_goal_along_trajectory = self.rough_trajectory.distance_to_end(
            i + t)
        return (distance_to_goal_along_trajectory -
                state.radius * 2.0) * self.heuristic_bias

    def goal(self):
        return self.next_goal

    def set_goal(self, pose):
        r = self.map.get_distances(np.array([pose]))[0]
        r = np.clip(r, 0.0, self.max_circle_radius)
        self.next_goal = Circle(
            center=pose[:2], radius=r, angle=pose[2], deflection=0)

    def overlap(self, s1, s2, percentage=.15):
        # TODO/NOTE: this is a bit of a hack to go fast, percentage overlap not accurate
        if np.linalg.norm(s1.center - s2.center) > (s1.radius + s2.radius) * (
                1.0 - percentage):
            return False
        else:
            return True

    def goal_met(self, state, goal_state):
        return self.overlap(state, goal_state, self.min_goal_overlap)

    def is_admissible(self, state):
        return state.radius > self.hard_min_radius

    def search(self, time_limit):
        # same as before, but this returns early if paths are found
        start_time = time.time()

        # extend nodes until the time limit is reached
        while time.time() - start_time < time_limit and not len(
                self.found_paths):
            if len(self.frontier) == 0:
                print("Search failed, bailing early")
                return
            self._step(neighbor_callback=self.add_states_to_exploration_buffer)

    def add_states_to_exploration_buffer(self, neighbor_circles):
        if len(neighbor_circles):
            poses = np.array(map(lambda c: c.center, neighbor_circles))
            radii = np.array(map(lambda c: c.radius, neighbor_circles))
            self.map.add_circles_to_exploration_buffer(
                poses, radii, exp_coeff=self.exploration_coeff)

    def neighbors(self, state):
        ''' This function determines the set of points along the perimeter of the given
		    circle state that are accessible according to ackermann geometry and steering
		    angle limitations. Then it samples the permissible theta space to generate neighbors.

		    TODO: computation of neighboring thetas is not accurate - it should actually be
		          the direction tangent to the path arc at the intersection with the circle radius.
		          For now this works, and is a strict underestimate of the reachable thetas
		'''
        max_angle = utils.max_angle(self.min_turn_radius, state.radius)
        percentage = np.power(2.0 * self.branch_factor / np.pi, 0.9)
        actual_branch_factor = 2 * int(max_angle * percentage / 2.0) + 1
        thetas = np.linspace(-max_angle, max_angle, num=actual_branch_factor)

        euclidean_neighbors = np.zeros((actual_branch_factor, 2))
        euclidean_neighbors[:, 0] = state.radius * np.cos(
            thetas + state.angle) + state.center[0]
        euclidean_neighbors[:, 1] = state.radius * np.sin(
            thetas + state.angle) + state.center[1]

        # perform coordinate space conversion here and then index into the exploration and permissible
        # buffers to prune states which are not permissible or already explored
        euc = euclidean_neighbors.copy()
        utils.world_to_map(euc, self.map.map_info)
        euc = np.round(euc).astype(int)
        radii = self.map.get_distances(
            euc, coord_convert=False, check_bounds=True) - 0.05
        radii = np.clip(radii, 0.0, self.max_circle_radius)
        mask = np.logical_and(
            np.invert(self.map.get_explored(euc, coord_convert=False)),
            self.map.get_permissible(euc, coord_convert=False))

        # generate a neighbor state for each of the permissible
        neighbors = map(lambda c,r,t: Circle(center=c.copy(), radius=r, angle=state.angle + t, deflection=t),
         euclidean_neighbors[mask,:], radii[mask], thetas[mask])
        return neighbors


class FindTrajectory(object):
    """ Performs two levels of heuristic search to plan paths between the robot pose and a clicked goal pose.
	"""

    def __init__(self):
        self.found_trajectory = utils.LineTrajectory("/found_trajectory")
        self.rough_trajectory = utils.LineTrajectory("/rough_trajectory")
        self.should_publish = bool(rospy.get_param("~publish"))
        self.pub_topic = rospy.get_param("~trajectory_topic")
        self.odom_topic = rospy.get_param("~odom_topic")
        self.exploration_timeout = rospy.get_param("~exploration_timeout")
        self.show_exploration_buffer = rospy.get_param(
            "~show_exploration_buffer")
        self.save_trajectory = rospy.get_param("~save_trajectory")
        self.save_path = rospy.get_param("~save_path")

        self.should_refine_trajectory = bool(
            rospy.get_param("~refine_trajectory"))
        self.refining_timeout = rospy.get_param("~refining_timeout")

        self.map = None
        self.map_initialized = False
        self.last_pose_time = 0.0
        self.circle_path = None

        # just a few initial poses for debugging purposes
        # self.last_pose = np.array([-1., 0., -np.pi/1.3])
        # self.last_pose = np.array([-1., 0., 0.0])
        self.last_pose = np.array([-3., -1., np.pi])
        # self.last_pose = np.array([-1.27, -2.6, np.pi])
        # self.last_pose = np.array([-6.27, -2.6, np.pi])
        # self.last_pose = np.array([15.3, 25.18, 0])
        # self.last_pose = np.array([-10,  33.8, 0])
        # self.last_pose = None

        self.get_omap()
        self.space_explorer = SpaceExploration(self.map)

        if self.should_publish:
            self.traj_pub = rospy.Publisher(
                self.pub_topic, navPath, queue_size=1)

        if self.should_refine_trajectory:
            self.path_planner = PathPlanner(self.map)
            self.fast_path = None
            self.fast_trajectory = utils.LineTrajectory("/fast_trajectory")

        # visualization publishers
        self.viz_namespace = "/circle_search"
        self.circle_pub = rospy.Publisher(
            self.viz_namespace + "/exploration_circles",
            MarkerArray,
            queue_size=1)
        self.fast_circle_pub = rospy.Publisher(
            self.viz_namespace + "/fast_circles", MarkerArray, queue_size=1)
        self.exploration_pub = rospy.Publisher(
            self.viz_namespace + "/exploration_buffer",
            OccupancyGrid,
            queue_size=1)

        self.nav_goal_sub = rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goal_point_callback,
            queue_size=1)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.cur_pose_sub = rospy.Subscriber(
            "/current_pose", PoseStamped, self.cur_pose_callback, queue_size=1)

        print "Initialized. Waiting on messages..."

    def get_omap(self):
        map_service_name = rospy.get_param("~static_map", "static_map")
        print "Fetching map from service: ", map_service_name
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        self.map = utils.Map(map_msg)
        self.map_initialized = True
        print "Finished loading map"

    def publish_trajectory(self):
        print "Publishing trajectory to:", self.pub_topic
        self.traj_pub.publish(self.rough_trajectory.toPath())

    def non_overlapping_paths(self, node):
        # given a tree data structure, this function will return list of lists of the
        # node "state" attributes
        # if visualized as a tree, these states will not contain overlapping segments
        if not node.children:
            return [node.state]
        else:
            paths = []

            for child in node.children:
                child_paths = self.non_overlapping_paths(child)

                if type(child_paths[0]) == list:
                    # child is not a leaf node, add self to first path and store
                    child_paths[0].insert(0, node.state)
                    paths = paths + child_paths
                else:
                    # this is a leaf node, add self to path and store
                    child_paths.insert(0, node.state)
                    paths.append(child_paths)

            return paths

    def visualize(self):
        if self.circle_pub.get_num_connections() > 0 and self.circle_path:
            print "CIRCLE PUB"
            # root = self.space_explorer.tree_root
            # markers = [utils.marker_clear_all("/map")]
            # explored = self.non_overlapping_paths(root)
            # if type(explored[0]) == list:
            # 	explored = reduce(lambda x,y: x+y, explored)
            # print "explored: ", len(explored)
            # markers += [utils.marker_from_circle(circle, index=i, linewidth=0.01, color=ColorRGBA(0, 1, 0, 0.1), lifetime=1.0) \
            #             for i, circle in enumerate(explored)]

            # marker_array = MarkerArray(markers=markers)
            # self.circle_pub.publish(marker_array)
            # print len(self.circle_path.states)
            # marker = utils.marker_from_circles(self.circle_path.states, 1, "Markers_NS2", "/map", 3.0, [1.,0.,0.])
            # self.circle_pub.publish(marker)

            markers = [utils.marker_clear_all("/map")]
            markers += [utils.marker_from_circle(circle, index=i, linewidth=0.05, color=ColorRGBA(1, 0, 0, 0.4), \
               lifetime=4.0)
               for i, circle in enumerate(self.circle_path.states)]
            # print len(markers)
            marker_array = MarkerArray(markers=markers)
            self.circle_pub.publish(marker_array)

        if self.should_refine_trajectory and self.fast_circle_pub.get_num_connections(
        ) > 0 and self.fast_path:
            print "FAST CIRCLE PUB"
            # root = self.path_planner.tree_root
            # markers = [utils.marker_clear_all("/map")]
            # explored = self.non_overlapping_paths(root)
            # if type(explored[0]) == list:
            # 	explored = reduce(lambda x,y: x+y, explored)
            # print "explored: ", len(explored)
            # markers += [utils.marker_from_circle(circle, index=i, linewidth=0.01, color=ColorRGBA(0, 0, 1, 0.1), lifetime=1.0) \
            #             for i, circle in enumerate(explored)]
            markers = [utils.marker_clear_all("/map")]
            markers += [utils.marker_from_circle(circle, index=i, linewidth=0.05, color=ColorRGBA(1, 0, 0, 0.4), \
               lifetime=4.0)
               for i, circle in enumerate(self.fast_path.states)]

            marker_array = MarkerArray(markers=markers)
            self.fast_circle_pub.publish(marker_array)

        if self.show_exploration_buffer and self.exploration_pub.get_num_connections(
        ) > 0:
            print "Publishing exploration grid"
            self.exploration_pub.publish(
                self.map.get_exploration_occupancy_grid())
        # plt.imshow(self.map.exploration_buffer)
        # plt.imshow(self.map.permissible_region)
        # plt.show()
        print "visualize"

    def goal_point_callback(self, msg):
        ''' Initialize path search. First performs and exploration search, and then a second search which uses
		    the result from the first search as a heuristic. Roughly respects dynamic constraints of the car.
		'''
        self.goal = np.array([
            msg.pose.position.x, msg.pose.position.y,
            utils.quaternion_to_angle(msg.pose.orientation)
        ])
        print "\nNew goal:", self.goal

        if self.has_recent_pose():
            # perform the first search
            self.find_rough_trajectory()
            if self.circle_path:
                self.rough_trajectory.clear()
                self.rough_trajectory.points = map(lambda x: x.center,
                                                   self.circle_path.states)
                self.rough_trajectory.update_distances()
                self.found_trajectory.points = self.rough_trajectory.points
                self.found_trajectory.publish_viz()

                if self.should_refine_trajectory:
                    self.refine_trajectory()
                    if self.fast_path:
                        self.fast_trajectory.clear()
                        self.fast_trajectory.points = map(
                            lambda x: x.center, self.fast_path.states)
                        self.fast_trajectory.update_distances()
                        self.found_trajectory.points = self.fast_trajectory.points

                if self.publish_trajectory:
                    self.traj_pub.publish(self.found_trajectory.toPath())

                if self.save_trajectory:
                    self.found_trajectory.save(
                        os.path.join(
                            self.save_path,
                            time.strftime("%Y-%m-%d-%H-%M-%S") + ".traj"))

                self.found_trajectory.update_distances()
                self.found_trajectory.publish_viz()
            self.visualize()
        else:
            print "No recent odometry, skipping search!"

    def find_rough_trajectory(self):
        print "Finding rough trajectory"
        self.space_explorer.set_goal(self.goal)
        self.space_explorer.reset(self.last_pose.copy())
        t = time.time()
        self.space_explorer.search(self.exploration_timeout)
        print "... rough search terminated in:", time.time() - t, "seconds"
        print "... expanded", self.space_explorer.step_count
        print "... considered", self.space_explorer.neighbor_count
        self.circle_path = self.space_explorer.best()
        if self.circle_path == None:
            print "...search failed"

    def refine_trajectory(self):
        print "Refining trajectory"
        self.rough_trajectory.make_np_array()
        self.path_planner.set_heursitic_trajectory(self.rough_trajectory)
        self.path_planner.set_goal(self.goal)
        self.path_planner.reset(self.last_pose.copy())

        t = time.time()
        self.path_planner.search(self.refining_timeout)
        print "... refine search terminated in:", time.time() - t, "seconds"
        print "... expanded", self.path_planner.step_count
        print "... considered", self.path_planner.neighbor_count
        self.fast_path = self.path_planner.best()
        if self.fast_path == None:
            print "... search failed"

    def has_recent_pose(self):
        # return True # this is useful for debugging
        return time.time() - self.last_pose_time < 0.5

    def odom_callback(self, msg):
        if not self.has_recent_pose():
            print "Recieved Odometry..."
        self.last_pose = np.array([
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            utils.quaternion_to_angle(msg.pose.pose.orientation)
        ])
        self.last_pose_time = time.time()

    def cur_pose_callback(self, msg):
        if not self.has_recent_pose():
            print "Recieved Pose..."
        self.last_pose = np.array([
            msg.pose.position.x, msg.pose.position.y,
            utils.quaternion_to_angle(msg.pose.orientation)
        ])
        self.last_pose_time = time.time()


def make_flamegraph(filterx=None):
    import flamegraph
    perf_log_path = os.path.join(os.path.dirname(__file__), "../tmp/perf2.log")
    flamegraph.start_profile_thread(
        fd=open(perf_log_path, "w"), filter=filterx, interval=0.001)


if __name__ == "__main__":
    rospy.init_node("trajectory_search")
    # make_flamegraph(r"goal_point_callback")
    pf = FindTrajectory()
    rospy.spin()

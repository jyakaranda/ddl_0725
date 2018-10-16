#!/usr/bin/env python

import numpy as np

angles = np.linspace(0.2, 20, 100)
# angles = np.copy(angles[0::18])
indices = np.array(range(10))
indices /= np.sum(indices)
size = np


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
    print 'diffs', diffs
    l2s = diffs[:, 0]**2 + diffs[:, 1]**2
    print 'l2s', l2s
    # this is equivalent to the elementwise dot product
    print ' ', (point - trajectories[:-1, :]) * diffs[:, :]
    dots = np.sum((point - trajectory[:-1, :]) * diffs[:, :], axis=1)
    print 'dots', dots
    t = np.clip(dots / l2s, 0.0, 1.0)
    print 't', t
    projections = trajectory[:-1, :] + (t * diffs.T).T
    print 'projections', projections
    dists = np.linalg.norm(point - projections, axis=1)
    print 'dists', dists
    min_dist_segment = np.argmin(dists)
    return projections[min_dist_segment], dists[min_dist_segment], t[
        min_dist_segment], min_dist_segment


point = np.array([2, 1], dtype=float)
trajectories = np.array([[0, 0], [1, 3], [1, 2], [2, 2]], dtype=float)
ls = [[0, 0], [1, 3], [1, 2], [2, 2]]
print trajectories[1][1]

import os
import numpy as np 
from algebraicTriangulation import *
from computeFundamentalMatrix import *
from decomposeEssentialMatrix import *


def normalize2Dpoints(points_2D):
	'''
	Normalize 2D points (also convert to Homogeneous Coordinates)

	Parameters
	----------

	points_2D: array-like, shape (num_points, 2)


	Returns
	----------

	norm_points: array-like, shape (3, num_points)
	'''

	# Dehomogenize if in homogeneous coordinates already

	if points_2D.shape[1] == 3:
		points_2D = points_2D[:,:2]

	mean_ = np.mean(points_2D, axis=0)
	dist = np.linalg.norm(points_2D - mean_)
	mean_dist = np.mean(np.squeeze(dist))

	scale = np.sqrt(2)/mean_dist

	T = [[scale,      0,      -scale*mean_[0]],
		 [0         scale,    -scale*mean_[1]],
		 [0,         0,                1    ]]

	return T*(points_2D-mean_).T
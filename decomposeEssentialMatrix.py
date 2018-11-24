import os
import numpy as np 
from random import *




def decomposeEssentialMatrix(E, x1, x2, K):

	'''
	Computing the relative orientation between two views by decomposing the essential 
	matrix and recovering R, t after cheirality testing. 


	Parameters
	----------

	E: array-like, shape (3, 3)
		
		essential matrix 		

	x1: array-like, shape (3, num_points)

	x2: array-like, shape (3, num_points)

		matched 2D point correspondences

	K: array-like, shape (3, 3)


	Returns
	----------

	R: array-like, shape (3, 3)

		ideal rotation matrix

	t: array-like, shape (1, 3)

		ideal translation vector

	'''

    # Convert to numpy arrays

    x1 = np.asarray(x1); x2 = np.asarray(x2); K = np.asarray(K); E = np.asarray(E)

    # Checking for incompatibilities

    if x1.shape[1] != x2.shape[1]:
        print "Both correspondences must have the same number of points"
        return 

    # Check if 2D points are in homogeneous coordinates

    if x1.shape[0] != 3 || x2.shape[0] != 3:
        print "2D points are not in homogeneous coordinates ---> Converting to homogeneous coordinates"

        if x1.shape[0] != 3:
            x1 = np.concat([x1, np.ones(1, x1.shape[1])])

        if x2.shape[0] != 3:
            x2 = np.concat([x2, np.ones(1, x2.shape[1])])


    # Defining 

    W = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    Z = [[0, 1, 0], [-1, 0, 0], [0, 0, 0]]

    # perform SVD of the essential matrix 

    U, D, V = np.linalg.svd(E)



import os
import numpy as np 
from algebraicTriangulation import *
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

    # Translation Guess
    t_guess = U(:,2)

    # Rotation Guess
    R_guess_1 = U*W*V.T
    R_guess_1 = R_guess_1*np.sign(np.linalg.det(R_guess_1))*np.sign(np.linalg.det(K))
    R_guess_2 = U*W.T*V.T
    R_guess_2 = R_guess_2*np.sign(np.linalg.det(R_guess_2))*np.sign(np.linalg.det(K))

    # Four possible solutions for the projection matrices
    P1 = K * np.asarray([np.eye(3), [0, 0, 0]])
    P21 = K * np.asarray([R_guess_1, t_guess])
    P22 = K * np.asarray([R_guess_1, -t_guess])
    P23 = K * np.asarray([R_guess_2, t_guess])
    P24 = K * np.asarray([R_guess_2, -t_guess])

    # We want to pick the camera poses for which most points are in front of both cameras (often called cheirality)

    X1 = algebraicTriangulation(x1, x2, P1, P21)
    X2 = algebraicTriangulation(x1, x2, P1, P22)
    X3 = algebraicTriangulation(x1, x2, P1, P23)
    X4 = algebraicTriangulation(x1, x2, P1, P24)

    # Converting the obtained 3D points in Homogeneous Coordinates

    X1 = X1/X1[4,:]
    X2 = X2/X2[4,:]
    X3 = X3/X3[4,:]
    X4 = X4/X4[4,:]


    # Compute image coordinates for all 3D point sets

    x1_1 = P1 * X1
    x2_1 = P21 * X1 

    x1_2 = P1 * X2 
    x2_2 = P22 * X2 

    x1_3 = P1 * X3 
    x2_3 = P23 * X3 

    x1_4 = P1 * X4
    x2_4 = P24 * X4 


    # Computing the third image coordinate and checking whether its positive and finite or not

    depth1_1 = x1_1[2,:]
    depth2_1 = x2_1[2,:]
    scores_1 = (depth1_1 > 0) + (depth2_1 > 0)
    scores_1 = np.sum(scores_1 == 2)

    depth1_2 = x1_2[2,:]
    depth2_2 = x2_2[2,:]
    scores_2 = (depth1_2 > 0) + (depth2_2 > 0)
    scores_2 = np.sum(scores_2 == 2)

    depth1_3 = x1_3[2,:]
    depth2_3 = x2_3[2,:]
    scores_3 = (depth1_3 > 0) + (depth2_3 > 0)
    scores_3 = np.sum(scores_3 == 2)

    depth1_4 = x1_3[2,:]
    depth2_4 = x2_3[2,:]
    scores_4 = (depth1_4 > 0) + (depth2_4 > 0)
    scores_4 = np.sum(scores_4 == 2)

















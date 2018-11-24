import os
import numpy as np 
from random import sample

def estimateFundamentalMatrix(matchedPoints1, matchedPoints2):
	'''

	Estimating Fundamental Matrix using the 2d point correspondences using the 8-points Algorithm

	Parameters
	----------

	matchedPoints1: array-like, shape (3, num_points)

	matchedPoints2: array-like, shape (3, num_points)
	
		matched 2D points correspondences
	


	Returns
	----------

	F: array-like, shape (3, 3)

		Estimated Fundamental Matrix
	

	'''

    matchedPoints1 = np.asarray(matchedPoints2); matchedPoints2 = np.asarray(matchedPoints2);

    # Checking for incompatibilities

    if matchedPoints1.shape[1] != matchedPoints2.shape[1]:
        print "Both correspondences must have the same number of points"
        return 

    # Check if 2D points are in homogeneous coordinates

    if matchedPoints1.shape[0] != 3 || matchedPoints2.shape[0] != 3:
        print "2D points are not in homogeneous coordinates ---> Converting to homogeneous coordinates"

        if matchedPoints1.shape[0] != 3:
            matchedPoints1 = np.concat([matchedPoints1, np.ones(1, matchedPoints1.shape[1])])

        if matchedPoints2.shape[0] != 3:
            matchedPoints2 = np.concat([matchedPoints2, np.ones(1, matchedPoints2.shape[1])])



    # Kronecker Multiplication
    a1 = np.multiply(matchedPoints1, np.repeat(matchedPoints2[:,0], [1,3]))
    a2 = np.multiply(matchedPoints1, np.repeat(matchedPoints2[:,1], [1,3]))

    A = np.asarray(list(a1) + list(a2) + list(matchedPoints1))

    # Do Singular Value Decomposition (SVD)
    U, S, V = np.linalg.svd(A)
    f = V[:,-1]
    F = np.reshape(f, (3,3))

    # Enforce Rank = 2

    U, S, V = np.linalg.svd(F)
    S[2,2] = 0
    F = U*S*V.T 

    return F




def estimateFundamentalMatrixRANSAC(matchedPoints1, matchedPoints2, maxIters=50):
	'''
	Estimating Fundamental Matrix using RANSAC to remove outliers

	Parameters
	----------

	matchedPoints1: array-like, shape (3, num_points)

	matchedPoints2: array-like, shape (3, num_points)

		matched 2D points correspondences

	maxIters: int


	Returns
	----------

	F: array-like, shape (3, 3)


	'''

	tol = 0.01
	num_points = matchedPoints1.shape[1]
	inliers = []  # stores inliers
	numInliers = -np.inf 

	for _ in range(maxIters):

		# Randomly sample 8 points
		idx = list(range(matchedPoints1.shape[1]))
		idx = sample(idx)
		x1 = matchedPoints1[:, idx]
		x2 = matchedPoints2[:, idx]

		# Getting the Fundamental Matrix estimate using the sampled points
		F_temp = estimateFundamentalMatrix(matchedPoints1[:, idx], matchedPoints2[:, idx])

		inliers_temp = []
		numInliers_temp = 0

		# Estimate the number of inliers

		for i in range(num_points):

			if np.abs(matchedPoints2[:, i].T*F_temp*matchedPoints1[:, i]) <= tol:
				inliers_temp.append(i)
				numInliers_temp += 1

			if numInliers_temp > numInliers:
				numInliers = numInliers_temp
				inliers = inliers_temp

			if numInliers >= 0.5*matchedPoints1.shape[1]:
				break


	# Estimate the fundamental matrix with highest number of inliers possible

	F = estimateFundamentalMatrix(matchedPoints1[:, inliers], matchedPoints2[:, inliers])

	return F






















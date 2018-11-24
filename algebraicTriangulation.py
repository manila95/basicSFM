import os
import numpy as np 


def algebraicTriangulation(x1, x2, P1, P2):
    ''' Triangulate 2d points correspondences into a 3D image using
    SVD 

    Parameters
    ----------

    x1: array-like, shape (3, n_points)

    x2: array-like, shape (3, n_points)

        2D points correspondences in homogeneous coordinates


    P1: array-like, shape (3, 4)

    P2: array-like, shape (3, 4)

        Projection matrices corresponding to the two camera poses


    Returns
    ----------

    pts_3D: array-like, shape (num_points, 4)

        Triangulated 3D points in homogeneous coordinates

    ''' 

    # Convert to numpy arrays

    x1 = np.asarray(x1); x2 = np.asarray(x2); P1 = np.asarray(P1); P2 = np.asarray(P2)


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


    num_points = x1.shape[1]

    pts_3D = []
    for i in range(num_points):
        a1 = x1[1,i]*P1[2,:] - P1[1,:]
        a2 = x1[0,i]*P1[2,:] - P1[0,:]
        a3 = x2[1,i]*P2[2,:] - P2[1,:]
        a4 = x2[0,i]*P2[2,:] - P2[0,:]


        # Solving Linear System Ax = 0 with least squares using SVD
        A = [a1, a2, a3, a4]
        U, S, V = np.linalg.svd(A, full_matrices=True)
        # Eigenvector corresponding to the smallest singular value (ideally zero/Null space eigen vector)
        X = V[:,-1]
        pts_3D.append(X/X[3])

    return pts_3D







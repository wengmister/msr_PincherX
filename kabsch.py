import numpy as np

def kabsch_algorithm(A, B):
    """
    Kabsch algorithm to find the optimal rotation and translation
    from set A to set B.
    
    :param A: Nx3 matrix representing N source points
    :param B: Nx3 matrix representing N target points
    :return: 4x4 homogeneous transformation matrix (HTM)
    """
    # Step 1: Calculate centroids of both sets
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Step 2: Center the points
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # Step 3: Compute the covariance matrix
    H = np.dot(A_centered.T, B_centered)

    # Step 4: Perform Singular Value Decomposition (SVD)
    U, S, Vt = np.linalg.svd(H)

    # Step 5: Compute the rotation matrix
    R = np.dot(Vt.T, U.T)

    # Step 6: Check for reflection and correct if necessary
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Step 7: Compute the translation vector
    t = centroid_B - np.dot(R, centroid_A)

    # Step 8: Construct the homogeneous transformation matrix (HTM)
    HTM = np.eye(4)
    HTM[:3, :3] = R  # Top-left 3x3 is the rotation matrix
    HTM[:3, 3] = t   # Top-right 3x1 is the translation vector

    return HTM

if __name__=="__main__":

    # Example usage:
    # Input points (Nx3)
    A = np.array([[1.0, 2.0, 3.0], 
                [4.0, 5.0, 6.0], 
                [7.0, 8.0, 9.0]])

    # Target points (Nx3)
    B = np.array([[1.1, 2.1, 3.1], 
                [3.9, 4.9, 6.1], 
                [6.9, 8.2, 9.1]])

    # Compute the HTM
    htm = kabsch_algorithm(A, B)


    print("Homogeneous Transformation Matrix (HTM):")
    print(htm)

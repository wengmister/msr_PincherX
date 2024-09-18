import numpy as np

def kabsch_algorithm(camera_points, target_points):
    # Step 1: Compute the centroids of both point sets
    camera_centroid = np.mean(camera_points, axis=0)
    target_centroid = np.mean(target_points, axis=0)

    # Step 2: Center the points around the centroids
    centered_camera_points = camera_points - camera_centroid
    centered_target_points = target_points - target_centroid

    # Step 3: Compute the covariance matrix (H)
    H = np.dot(centered_camera_points.T, centered_target_points)

    # Step 4: Compute the SVD of the covariance matrix
    U, S, Vt = np.linalg.svd(H)

    # Step 5: Compute the rotation matrix
    R = np.dot(Vt.T, U.T)

    # Step 6: Ensure the rotation matrix is proper (determinant = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Step 7: Compute the translation vector
    t = target_centroid - np.dot(R, camera_centroid)

    # Step 8: Build the homogeneous transformation matrix (HTM)
    htm = np.eye(4)
    htm[:3, :3] = R
    htm[:3, 3] = t

    return htm

# Example usage:
camera_points = np.array([[x1, y1, z1], [x2, y2, z2], ..., [xn, yn, zn]])  # Replace with actual points
target_points = np.array([[x1', y1', z1'], [x2', y2', z2'], ..., [xn', yn', zn']])  # Replace with actual points

htm = kabsch_algorithm(camera_points, target_points)
print("Homogeneous Transformation Matrix:")
print(htm)

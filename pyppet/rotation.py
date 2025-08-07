import numpy as np


def rotation_matrix_from_euler(euler_angles: tuple[float, float, float]):
    """Compute the rotation matrix from Euler angles (XYZ convention)."""

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(euler_angles[0]), -np.sin(euler_angles[0])],
                    [0, np.sin(euler_angles[0]), np.cos(euler_angles[0])]])
    R_y = np.array([[np.cos(euler_angles[1]), 0, np.sin(euler_angles[1])],
                    [0, 1, 0],
                    [-np.sin(euler_angles[1]), 0, np.cos(euler_angles[1])]])
    R_z = np.array([[np.cos(euler_angles[2]), -np.sin(euler_angles[2]), 0],
                    [np.sin(euler_angles[2]), np.cos(euler_angles[2]), 0],
                    [0, 0, 1]])

    return np.dot(R_z, np.dot(R_y, R_x))  # Combine the rotation matrices (Z * Y * X order)

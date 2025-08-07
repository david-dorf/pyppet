from pyppet.rotation import rotation_matrix_from_euler
import numpy as np


def test_identity_rotation():
    euler_angles = (0, 0, 0)
    expected_matrix = np.eye(3)
    result_matrix = rotation_matrix_from_euler(euler_angles)
    np.testing.assert_allclose(result_matrix, expected_matrix, atol=1e-7)

def test_90_degree_rotation_x():
    euler_angles = (np.pi / 2, 0, 0)
    expected_matrix = np.array([[1, 0, 0],
                                [0, 0, -1],
                                [0, 1, 0]])
    result_matrix = rotation_matrix_from_euler(euler_angles)
    np.testing.assert_allclose(result_matrix, expected_matrix, atol=1e-7)

def test_90_degree_rotation_y():
    euler_angles = (0, np.pi / 2, 0)
    expected_matrix = np.array([[0, 0, 1],
                                [0, 1, 0],
                                [-1, 0, 0]])
    result_matrix = rotation_matrix_from_euler(euler_angles)
    np.testing.assert_allclose(result_matrix, expected_matrix, atol=1e-7)

def test_90_degree_rotation_z():
    euler_angles = (0, 0, np.pi / 2)
    expected_matrix = np.array([[0, -1, 0],
                                [1, 0, 0],
                                [0, 0, 1]])
    result_matrix = rotation_matrix_from_euler(euler_angles)
    np.testing.assert_allclose(result_matrix, expected_matrix, atol=1e-7)

def test_90_degree_rotation_xyz():
    euler_angles = (np.pi / 2, np.pi / 2, np.pi / 2)
    expected_matrix = np.array([[-0.5, -0.5, 0.70710678],
                                [0.5, -0.5, 0.70710678],
                                [-0.70710678, 0.70710678, 0]])
    result_matrix = rotation_matrix_from_euler(euler_angles)
    np.testing.assert_allclose(result_matrix, expected_matrix, atol=1e-7)

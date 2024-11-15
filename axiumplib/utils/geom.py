import numpy as np
from math import cos, sin, sqrt
from axiumplib.utils.decorators import memoize


@memoize
def normalize_2D_vector(x, y):
    """Normalize a vector."""
    magn = sqrt(x**2 + y**2)
    if magn != 0:
        normalized_vector = [x / magn, y / magn]
    else:
        normalized_vector = [0, 0]
    return normalized_vector, magn


def wrap_point_on_X_axis_cylinder(point, radius):
    """Wrap a 2D point on a cylinder around the X axis."""
    assert radius != 0, "Radius must be greater than 0."
    theta = point[1] / radius
    return [point[0], radius * sin(theta), radius * cos(theta)]


def wrap_point_on_cylinder(point, axis, radius, origin=[0, 0]):
    """Wrap a 2D point on a cylinder, given a 2D axis and an origin."""
    assert radius != 0, "Radius must be greater than 0."
    axis, _ = normalize_2D_vector(*axis)
    point = np.array(point)
    axis = np.array(axis)
    origin = np.array(origin)
    point = point - origin

    proj_point = point.dot(axis) * axis
    dist_vec = point - proj_point
    dist_vec, dist = normalize_2D_vector(*dist_vec)
    dist_vec = np.array(dist_vec)

    theta = dist / radius
    wrapped_point = proj_point + radius * sin(theta) * dist_vec
    wrapped_point = wrapped_point + origin
    wrapped_point = np.concatenate([wrapped_point, [radius * cos(theta)]])

    return wrapped_point


@memoize
def get_2D_rotation_matrix(alpha):
    """Get a 2D rotation matrix."""
    cos_alpha = cos(alpha)
    sin_alpha = sin(alpha)
    rotation_matrix = [[cos_alpha, -sin_alpha], [sin_alpha, cos_alpha]]

    return rotation_matrix


def mat_dot_vec_2D(matrix, vector):
    """Dot product of a 2x2 matrix and a 2D vector."""
    return [matrix[0][0] * vector[0] + matrix[0][1] * vector[1], matrix[1][0] * vector[0] + matrix[1][1] * vector[1]]


def rotate_2D_pt(point, alpha):
    """Rotate a 2D point by angle alpha."""
    rotmat = get_2D_rotation_matrix(alpha)
    rotated_point = mat_dot_vec_2D(rotmat, point)

    return rotated_point


def rotate_2D_pts(points, alpha):
    """
    Rotate 2D points by angle alpha (value, or array).
    Takes any array following the order x1 y1 x2 y2... and returns the same shape.
    """
    points = np.array(points)
    shape = points.shape
    points = points.reshape(-1, 2)

    if isinstance(alpha, (int, float)):
        alpha = np.full(points.shape[0], alpha)
    alpha = np.array(alpha)

    rotation_matrix = get_2D_rotation_matrix(alpha)
    rotated_points = np.einsum("jki,ik->ij", rotation_matrix, points)

    return rotated_points.reshape(shape)

import numpy as np
import math


def latlong_to_3d(latitude_r, longitude_r):
    """
    Convert a point given latitude and longitude in radians to 3-dimensional space, assuming a
    sphere radius of one.
    """
    return np.array((
        math.cos(latitude_r) * math.cos(longitude_r),
        math.cos(latitude_r) * math.sin(longitude_r),
        math.sin(latitude_r)
    ))


def angle_between_vectors_degrees(u, v):
    """
    Return the angle between two vectors in any dimension space, in degrees.
    """
    return np.degrees(math.acos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))))


def in_2d(lat1, lon1, lat2, lon2, lat3, lon3):
    """
    Find the angle between the vectors in 2D space
    The points are in tuple latitude/longitude degrees
    https://stackoverflow.com/questions/42584259/python-code-to-calculate-angle-between-three-points-lat-long-coordinates
    """

    # Convert the points to numpy latitude/longitude radians space
    point_a = np.radians(np.array((lat1, lon1)))
    point_b = np.radians(np.array((lat2, lon2)))
    point_c = np.radians(np.array((lat3, lon3)))

    # Vectors in latitude/longitude space
    vector_a = point_a - point_b
    vector_c = point_c - point_b

    # Adjust vectors for changed longitude scale at given latitude into 2D space
    lat = point_b[0]
    vector_a[1] *= math.cos(lat)
    vector_c[1] *= math.cos(lat)

    # Return the angle between the vectors in 2D space
    return angle_between_vectors_degrees(vector_a, vector_c)

#https://stackoverflow.com/questions/42584259/python-code-to-calculate-angle-between-three-points-lat-long-coordinates

import numpy as np
import math

def latlong_to_3d(latr, lonr):
    """Convert a point given latitude and longitude in radians to
    3-dimensional space, assuming a sphere radius of one."""
    return np.array((
        math.cos(latr) * math.cos(lonr),
        math.cos(latr) * math.sin(lonr),
        math.sin(latr)
    ))

def angle_between_vectors_degrees(u, v):
    """Return the angle between two vectors in any dimension space,
    in degrees."""
    return np.degrees(math.acos(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v))))

def in_2d(lat1, lon1, lat2, lon2, lat3, lon3): # The points in tuple latitude/longitude degrees space
    A = (lat1, lon1)
    B = (lat2, lon2)
    C = (lat3, lon3)

    # Convert the points to numpy latitude/longitude radians space
    a = np.radians(np.array(A))
    b = np.radians(np.array(B))
    c = np.radians(np.array(C))

    # Vectors in latitude/longitude space
    avec = a - b
    cvec = c - b

    # Adjust vectors for changed longitude scale at given latitude into 2D space
    lat = b[0]
    avec[1] *= math.cos(lat)
    cvec[1] *= math.cos(lat)

    # Find the angle between the vectors in 2D space
    angle2deg = angle_between_vectors_degrees(avec, cvec)

    # The points in 3D space
    a3 = latlong_to_3d(*a)
    b3 = latlong_to_3d(*b)
    c3 = latlong_to_3d(*c)

    # Vectors in 3D space
    a3vec = a3 - b3
    c3vec = c3 - b3

    # Find the angle between the vectors in 2D space
    angle3deg = angle_between_vectors_degrees(a3vec, c3vec)

    return angle2deg

    # Print the results
    #print('\nThe angle ABC in 2D space in degrees:', angle2deg)
    #print('\nThe angle ABC in 3D space in degrees:', angle3deg)
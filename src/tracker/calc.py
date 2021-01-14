from collections import namedtuple
from math import atan2, cos, sin, asin, radians, degrees, sqrt

# R is earth radius
R = 6371  # in km
waypoint = namedtuple('waypoint', 'latitude longitude')


def find_bearing(point_a, point_b):
    """
    Find bearing between two waypoints
    (Algorithm:http://www.movable-type.co.uk/scripts/latlong.html (x and y are inverted))
    """
    return (atan2((cos(radians(point_b.latitude)) * (sin(radians(point_b.longitude) - radians(point_a.longitude)))),
                  (cos(radians(point_a.latitude)) * sin(radians(point_b.latitude)) - (sin(radians(point_a.latitude)) *
                                                                                      cos(radians(point_b.latitude)) *
                                                                                      cos(radians(point_b.longitude) -
                                                                                          radians(
                                                                                              point_a.longitude))))))


def find_command_waypoint(dis, bear, starting_waypoint):
    """
    find destination point given a distance and and bearing from a starting point
    """
    d = dis / R  # d is the angular distance
    bearing = radians(bear)
    target_latitude = asin((sin(radians(starting_waypoint.latitude)) * cos(d)) +
                           (cos(radians(starting_waypoint.latitude)) * sin(d) * cos(bearing)))

    target_longitude = radians(starting_waypoint.longitude) + atan2(sin(bearing) * sin(d) *
                                                                    cos(radians(starting_waypoint.latitude)),
                                                                    cos(d) - sin(radians(starting_waypoint.latitude)) *
                                                                    sin(target_latitude))

    return waypoint(latitude=round(degrees(target_latitude), 7), longitude=round(degrees(target_longitude), 7))


def get_closest_point_on_segment(sx1, sy1, sx2, sy2, px, py):
    """
    Getting closest point to segment
    (Algorithm: http://www.java2s.com/Code/Java/2D-Graphics-GUI/Returnsclosestpointonsegmenttopoint.htm
    """

    x_delta = sx2 - sx1
    y_delta = sy2 - sy1
    u = ((px - sx1) * x_delta + (py - sy1) * y_delta) / (x_delta * x_delta + y_delta * y_delta)
    if u == 0:
        return waypoint(latitude=round((sx1 + u * x_delta), 7), longitude=round((sy1 + u * y_delta), 7))
    else:
        return waypoint(latitude=sx1, longitude=sy1) if (u < 0) else waypoint(latitude=sx2, longitude=sy2)


def get_distance(lat1, lon1, lat2, lon2):
    """
    Getting distance between two points
    """
    a = (sin(radians(lat2 - lat1) / 2) * sin(radians(lat2 - lat1) / 2)) + (cos(radians(lat1)) * cos(radians(lat2)) *
                                                                           sin(radians(lon2 - lon1) / 2) *
                                                                           sin(radians(lon2 - lon1) / 2))
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

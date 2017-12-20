from collections import namedtuple
from math import atan2, cos, sin, asin, radians, degrees, sqrt

# R is earth radius
R = 6371 # in km
waypoint = namedtuple('waypoint', 'latitude longitude')

# find bearing between two waypoints
# http://www.movable-type.co.uk/scripts/latlong.html (x and y are inverted)
def find_bearing(a, b):

    x_value = cos(radians(b.latitude)) * ( sin(radians(b.longitude) - radians(a.longitude)) )
    y_value = cos(radians(a.latitude)) * sin(radians(b.latitude)) - (sin(radians(a.latitude))  * cos(radians(b.latitude)) * cos(radians(b.longitude) - radians(a.longitude)))
    return degrees(atan2(x_value, y_value))

# find destination point given a distance and and bearing from a starting point
def find_command_waypoint(dis, bear, starting_waypoint):

    # d is the angular distance
    d = dis / R
    bearing = radians(bear)
    target_latitude = asin( (sin(radians(starting_waypoint.latitude)) * cos(d) ) + \
                            (cos(radians(starting_waypoint.latitude)) * sin(d) * cos(bearing)))
    target_longitude = radians(starting_waypoint.longitude) + \
                atan2(sin(bearing) * sin(d) * cos(radians(starting_waypoint.latitude)),
                      cos(d) - sin(radians(starting_waypoint.latitude)) * sin(target_latitude))
    return waypoint(latitude=round(degrees(target_latitude),7), longitude=round(degrees(target_longitude),7))

#http://www.java2s.com/Code/Java/2D-Graphics-GUI/Returnsclosestpointonsegmenttopoint.htm
def get_closest_point_on_segment(sx1, sy1, sx2, sy2, px, py):

    xDelta = sx2 - sx1
    yDelta = sy2 - sy1
    u = ((px - sx1) * xDelta + (py - sy1) * yDelta) / (xDelta * xDelta + yDelta * yDelta)

    if u < 0:
      closestPoint = waypoint(sx1, sy1)
    elif u > 1:
      closestPoint = waypoint(sx2, sy2)
    else:
      closestPoint = waypoint(round((sx1 + u * xDelta),7), round((sy1 + u * yDelta),7))

    return closestPoint

def get_distance(lat1, lon1, lat2, lon2):

    f1 = radians(lat1)
    f2 = radians(lat2)
    df = radians(lat2-lat1)
    dl = radians(lon2-lon1)

    a = ( sin(df/2) * sin(df/2) ) + ( cos(f1) * cos(f2) * sin(dl/2) * sin(dl/2) )
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return R * c

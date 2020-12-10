#!/usr/bin/env python

import rospy
import numpy as np
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees
from tf.transformations import euler_from_quaternion



def injectPoints(waypoints, space):
    spacing = space
    new_points = []
    for j in range(0, len(waypoints) - 1):
        start_point = waypoints[j]
        end_point = waypoints[j + 1]
        vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
        d = sqrt(pow(vector[0], 2) + pow(vector[1], 2))
        num_points_that_fit = int(ceil(d / spacing))
        vector = (vector[0] / d * spacing, vector[1] / d * spacing)
        for i in range(0, num_points_that_fit):
            new_list = (start_point[0] + vector[0] * i, start_point[1] + vector[1] * i)
            new_points.append(new_list)
        new_points.append(end_point)
    return new_points


def smoothPath(path):  # path is [(x1, y1), ..., (xend, yend)]
    b = 0.99 #0.75
    a = 1 - b
    tolerance = 0.001
    newPath = [list(point) for point in path]  # tuples are immutable
    change = tolerance
    while change >= tolerance:
        change = 0
        for i in range(1, len(path) - 1):
            for j in range(0, len(path[i])):
                aux = newPath[i][j]
                newPath[i][j] += a * (path[i][j] - newPath[i][j]) + b * (newPath[i - 1][j] + newPath[i + 1][j]
                                                                         - (2.0 * newPath[i][j]))
                change += abs(aux - newPath[i][j])
    newPath = list(tuple(point) for point in newPath)
    return newPath


def main():
    angle = radians(45)  # in radians
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10 * cos(angle), 10 * sin(angle))
    waypoints = [branching_point, end_point]
    waypoints2 = [(0, 0), branching_point, end_point]
    #waypoints = [(10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    #waypoints2 = [(0,0), (10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    path = injectPoints(waypoints2, 0.1)
    smooth_path = smoothPath(path)
    print(path)
    print(smooth_path)


if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    main()

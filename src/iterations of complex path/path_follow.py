#!/usr/bin/env python

import sys  # for redirecting output in bash, could be removed
# import time # for sleeping - time.sleep is commented out below right now
import os
import rospy
import argparse
import subprocess
from subprocess import Popen
import psutil
import time
from geometry_msgs.msg import Twist
import numpy as np
#import keras
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0
yaw2 = 0.0
quat = 0.0
quat2 = 0.0


def statesCallback(data):
    global x, y, v, yaw, quat
    # find index of slash
    name = data.name
    index = name.index("jackal")
    # index = name.index("/")
    x = data.pose[index].position.x
    y = data.pose[index].position.y
    v = data.twist[index].linear.x
    quaternion = (
        data.pose[index].orientation.x,
        data.pose[index].orientation.y,
        data.pose[index].orientation.z,
        data.pose[index].orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    quat = quaternion[3]


def odomCallback(data):
    global yaw2, quat2
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w
    )
    euler = euler_from_quaternion(quaternion)
    yaw2 = euler[2]
    quat2 = quaternion[3]



def robotUnsafe(robx, roby, path, tol):
    safety_tolerance = tol
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i + 1
    val = min(dists)
    closest_index = dists.index(val)
    return val > safety_tolerance, val


def robotAtGoal(robx, roby, goalx, goaly):
    distance_tolerance = 1
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def getLookAheadPoint(path, robx, roby, lookAheadDistance, space, lastLookAhead):
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i + 1
    val = min(dists)
    closest_index = dists.index(val)
    lookAheadIndex = closest_index + int(lookAheadDistance/space)
    if lookAheadIndex <= lastLookAhead:
        lookAheadIndex = lastLookAhead + 1
    if lookAheadIndex > (len(path)-1):
        lookAheadIndex = len(path)-1
    lookAheadPoint = path[lookAheadIndex]
    return lookAheadPoint, closest_index, lookAheadIndex


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
    b = 0.9 #0.75
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


def main(safety_tol):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    info = "{lin_vel}, {ang_vel}, {angle}, {deviation}\n"
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(30)  # in radians
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10 * cos(angle), 10 * sin(angle))
    #waypoints = [branching_point, end_point]
    #waypoints2 = [(0, 0), branching_point, end_point]
    waypoints = [(10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    waypoints2 = [(0,0), (10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    space = 0.1
    path = injectPoints(waypoints2, 0.1)
    smooth_path = smoothPath(path)
    lookAheadDistance = 2.9
    lookAheadPoint = waypoints[0]
    lastLookAhead = 0
    atGoalHack = 0
    #print(path)


    while not rospy.is_shutdown():
        path = injectPoints(waypoints2, 0.1)
        unsafe, robot_deviation = robotUnsafe(x, y, smooth_path, safety_tol)
        if unsafe:
            print("unsafe")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastLookAhead == len(path) - 1:
            print("at goal:", x, y)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break
       
        lookAheadPoint, closest_index, lookAheadIndex = getLookAheadPoint(path, x, y, lookAheadDistance, space, lastLookAhead)
        lastLookAhead = lookAheadIndex
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        vel = 2
        # linear velocity in the x-axis:
        vel_msg.linear.x = vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)
        print(x, y, yaw, quat, lookAheadIndex, lookAheadPoint, atan2(goal_pose_y - y, goal_pose_x - x), vel)
        #print(goal_pose_x, goal_pose_y)

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

    # log_file.close()
    print("kill me")
    sys.stdout.flush()
    # time.sleep(20)
    raw_input("")  # kill 0 sent from bash script not working, so you have to ctrl-c manually

    for process in psutil.process_iter():
        print(process.cmdline())


if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    safety_tol = 2
    main(safety_tol)

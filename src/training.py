#!/usr/bin/env python

import sys
import os
import rospy
import datetime
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians
from tf.transformations import euler_from_quaternion
import argparse
import subprocess
from subprocess import Popen
import psutil
import time
import csv

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0


def statesCallback(data):
    global x, y, v, yaw
    # find index of jackal
    name = data.name
    index = name.index("jackal")
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


def calculate_mu(run):
    if run <= 120:
        return 0.009
    elif 121 < run <= 240:
        return 0.09
    elif 241 < run <= 360:
        return 1
    elif 361 < run <= 480:
        return 0.05
    elif 481 < run:
        return 0.5
    return None


def calculate_velocity(run):
    if run % 10 == 1:
        return 0.2
    elif run % 10 == 2:
        return 0.4
    elif run % 10 == 3:
        return 0.6
    elif run % 10 == 4:
        return 0.8
    elif run % 10 == 5:
        return 1.0
    elif run % 10 == 6:
        return 1.2
    elif run % 10 == 7:
        return 1.4
    elif run % 10 == 8:
        return 1.6
    elif run % 10 == 9:
        return 1.8
    elif run % 10 == 0:
        return 2.0
    return None


def robotUnsafe(robx, roby, path, safety_tolerance):
    dists = [0]*len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i+1
    val = min(dists)
    return val > safety_tolerance, val


def robotAtGoal(robx, roby, goalx, goaly):
    distance_tolerance = 1
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def getLookAheadPoint(path, robx, roby, lastLookAhead):
    dx = [robx - pathx[0] for pathx in path]
    dy = [roby - pathy[1] for pathy in path]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d) + 30
    #if target_idx <= lastLookAhead:
    #    target_idx = lastLookAhead + 1
    if target_idx > (len(path) - 1):
        target_idx = (len(path) - 1)
    return target_idx


def injectPoints(waypoints):
    spacing = 0.1
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
    b = 0.9
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


def stop_robot(vel_msg, velocity_publisher):
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def main(velocity, angle_deg, safety_threshold):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(angle_deg)  # in radians
    branching_point = (15, 0)
    end_point = (branching_point[0] + 15*cos(angle), 15*sin(angle))
    print(end_point)
    waypoints = [branching_point, end_point]
    waypoints2 = [(0, 0), branching_point, end_point]
    path = injectPoints(waypoints2)
    lastLookAhead = 0
    atGoalHack = 0  # needs to be fixed
    csvinput = []
    filename = "training_old.csv"


    begin = datetime.datetime.now()
    time_to_stop = 4  # in minutes

    while not rospy.is_shutdown():
        path = injectPoints(waypoints2)
        unsafe, robot_deviation = robotUnsafe(x, y, path, safety_threshold)
        if unsafe:
            print("unsafe")
            stop_robot(vel_msg, velocity_publisher)
            break

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastLookAhead == len(path) - 1:
            print("at goal:", x, y)
            stop_robot(vel_msg, velocity_publisher)
            break

        now = datetime.datetime.now()
        # will stop program if robot hasn't found goal or become unsafe after time_to_stop minutes
        if begin + datetime.timedelta(minutes=time_to_stop) < now:
            print("timed out")
            stop_robot(vel_msg, velocity_publisher)
            break

        target_index = getLookAheadPoint(path, x, y, lastLookAhead)
        lookAheadPoint = path[target_index]
        lastLookAhead = target_index  # lookAheadIndex
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        theta_d = atan2(goal_pose_y - y, goal_pose_x - x)
        theta_diff = theta_d - yaw
        ang_vel = atan2(sin(theta_diff), cos(theta_diff))

        if ang_vel < -(pi / 2):
            ang_vel = -pi / 2
        elif ang_vel > pi / 2:
            ang_vel = pi / 2

        # linear velocity in the x-axis:
        vel_msg.linear.x = velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4*ang_vel

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

        csvinput.append([x, y, yaw, target_index, goal_pose_x, goal_pose_y, velocity, angle_deg, ang_vel])
        with open(filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow([x, y, yaw, target_index, goal_pose_x, goal_pose_y, velocity, angle_deg, ang_vel])

    #print("Killing now...")
    print("Done.")
    sys.stdout.flush()
    
    os.popen('killall -9 rosmaster')
    os.popen('killall -9 roscore')
    os.popen('killall -9 gzclient')
    os.popen('killall -9 gzserver')


if __name__ == "__main__":
    rospy.init_node('training_nodes', anonymous=True)

    run = 1210 #rospy.get_param('~run')
    angle = 120 #rospy.get_param('~angle')
    safety_threshold = 4
    mu = 0.09 #calculate_mu(run)
    velocity = 2 #calculate_velocity(run)
    #velocity = 1
    #angle = 90
    #mu = 0.009
    print("velocity: ", velocity, "angle: ", angle)
    main(velocity, angle, safety_threshold)



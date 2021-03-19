#!/usr/bin/env python

import sys  # for redirecting output in bash, could be removed
import time  # for sleeping - time.sleep is commented out below right now
import os
import rospy
import argparse
import subprocess
from subprocess import Popen
import psutil
from geometry_msgs.msg import Twist
import numpy as np
import keras
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees, exp, log
from tf.transformations import euler_from_quaternion
import csv

x = 0.0
y = 0.0
v = 0.0
yaww = 0.0
yaw2 = 0.0
quat = 0.0
quat2 = 0.0

environments = {0.009: "ice_009",
                0.09: "ice_09",
                0.9: "ice_9",
                1: "control",
                1000: "mud",
                0.05: "ice_05",
                0.5: "ice_5",
                0.02: "ice_02",
                0.2: "ice_2",
                0.07: "ice_07",
                0.7: "ice_7",
                0.005: "ice_005",
                0.0009: "ice_0009"}

model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/2_speed_network(feb1).h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)


def statesCallback(data):
    global x, y, v, yaww, quat
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
    yaww = euler[2]
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


def robotAtGoal(robx, roby, goalx, goaly, tol):
    distance_tolerance = tol
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def getClosestIndex(robx, roby, path, tol):
    safety_tolerance = tol
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i + 1
    val = min(dists)
    closest_index = dists.index(val)
    return closest_index


def getLookAheadPoint(path, robx, roby, lastLookAhead):
    dx = [robx - pathx[0] for pathx in path]
    dy = [roby - pathy[1] for pathy in path]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d) + 30
    # if target_idx <= lastLookAhead:
    #   target_idx = lastLookAhead + 1
    if target_idx > (len(path) - 1):
        target_idx = (len(path) - 1)
    return target_idx


def getLookAheadPoint2(path, robx, roby, space, lastLookAhead):
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i + 1
    val = min(dists)
    closest_index = dists.index(val)
    closest_point = path[closest_index]
    d = [10] * len(path)
    j = 0
    # print(len(path))
    for point in path:
        # print(j)
        d[j] = (sqrt(pow((point[0] - closest_point[0]), 2) + pow((point[1] - closest_point[1]), 2))) - 4
        if d[j] < 0:
            d[j] = 10
        j = j + 1
    val2 = min(d)
    print(val2)
    lookAheadIndex = d.index(val2)
    if lookAheadIndex <= lastLookAhead:
        lookAheadIndex = lastLookAhead + 1
    if lookAheadIndex > (len(path) - 1):
        lookAheadIndex = len(path) - 1
    lookAheadPoint = path[lookAheadIndex]
    return lookAheadPoint, lookAheadIndex


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
    b = 0.9  # 0.75
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


def checkAlignment(closest_index, horizon, x, y, last_x, last_y, path_predict, tol):
    try:
        horizon_point1 = path_predict[closest_index + horizon]
        horizon_point2 = path_predict[closest_index + horizon + 1]
    except IndexError as e:
        horizon_point1 = path_predict[-2]
        horizon_point2 = path_predict[-1]
    path_angle = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])
    yaw = atan2(y - last_y, x - last_x)
    a = path_angle - yaw
    ang = abs(degrees(a))
    return ang - tol < 0, ang


def calculate_lookahead(mu):
    if mu >= 0.2:
        return 5
    elif 0.2 < mu <= 0.05:
        return 10
    else:
        return 20


def main(mu, safety_tol):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(160)  # in radians
    branching_point = (15, 0)
    end_point = (branching_point[0] + 15 * cos(angle), 15 * sin(angle))
    waypoints = [branching_point, end_point]
    waypoints2 = [(0, 0), branching_point, end_point]
    # waypoints = [(20,0), (20, 20), (0, 20), (0,0)]
    # waypoints2 = [(0,0), (20,0), (20, 20), (0, 20), (0,0)]
    # waypoints = [(10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    # waypoints2 = [(0,0), (10,0), (20, 10), (30, 10), (10, 20), (0,0)]
    space = 0.1
    path = injectPoints(waypoints2, space)
    path_predict = injectPoints(waypoints2, 0.5)
    smooth_path = smoothPath(path)
    atGoalHack = 0
    # pred_vels = [0] * 4
    lookahead_num = calculate_lookahead(mu)
    stored_angs = [0] * lookahead_num
    filename = "run_data_dynamic.csv"
    lastLookAhead = 0
    # lookAheadPoint, lookAheadIndex = getLookAheadPoint2(path, x, y, space, lastLookAhead)
    # lastLookAhead = lookAheadIndex
    lookAheadIndex = getLookAheadPoint(path, x, y, lastLookAhead)
    lookAheadPoint = path[lookAheadIndex]
    goal_pose_x = lookAheadPoint[0]
    goal_pose_y = lookAheadPoint[1]
    lastLookAhead = lookAheadIndex
    last_x = 0
    last_y = 0
    last_vel = 0
    csvinput = []
    print("lookahead:", lookahead_num, "angle:", degrees(angle))
    with open(filename, 'a') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerow(['feb16', 'new', 'new', 'new', 'new', 'new', 'new', 'new', 'new', 'new', 'new', 'new'])

    while not rospy.is_shutdown():

        path = injectPoints(waypoints2, space)

        unsafe, robot_deviation = robotUnsafe(x, y, smooth_path, safety_tol)
        if unsafe:
            print("unsafe")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1], 1) and lastLookAhead == len(path) - 1:
            print("at goal:", x, y)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            break

        # if robotAtGoal(x, y, goal_pose_x, goal_pose_y, 1):
        # lookAheadPoint, lookAheadIndex = getLookAheadPoint(path, x, y, space, lastLookAhead)

        lookAheadIndex = getLookAheadPoint(path, x, y, lastLookAhead)
        lookAheadPoint = path[lookAheadIndex]
        lastLookAhead = lookAheadIndex
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        closest_index = getClosestIndex(x, y, path_predict, safety_tol)
        

        horizon = 0
        while horizon < lookahead_num:
            try:
                horizon_point1 = path_predict[closest_index + horizon]
                horizon_point2 = path_predict[closest_index + horizon + 1]
            except IndexError as e:
                horizon_point1 = path_predict[-2]
                horizon_point2 = path_predict[-1]
            # a = abs(atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])) - abs(yaw)
            path_angle = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])
            x_store = x
            y_store = y
            last_y_store = last_y
            last_x_store = last_x
            yaw = atan2(y - last_y, x - last_x)
            a = path_angle - yaw
            ang = abs(degrees(a))
            stored_angs[horizon] = ang
            horizon = horizon + 1
        
        last_x = x
        last_y = y
        #print(stored_angs)
        big_ang = max(stored_angs)
        velo = model.predict([[mu, big_ang, safety_tol]])[0][1]
        # Current Measure of Safety is slowest but how will that be with more complex systems
        #bol, align_ang = checkAlignment(closest_index, horizon, x, y, last_x, last_y, path_predict, 15)
        #if (bol or velo < last_vel):
        vel = velo
        align_ang = 0
        #else:
        #    vel = last_vel
        print(vel, abs(v))

        theta_d = atan2(goal_pose_y - y, goal_pose_x - x)
        theta_diff = theta_d - yaww
        ang_vel = atan2(sin(theta_diff), cos(theta_diff))

        if ang_vel < -(pi / 2):
            ang_vel = -pi / 2
        elif ang_vel > pi / 2:
            ang_vel = pi / 2

        # linear velocity in the x-axis:
        vel_msg.linear.x = vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 4 * ang_vel

        csvinput.append([x_store, y_store, yaw, lookAheadIndex, goal_pose_x, goal_pose_y, vel, ang, ang_vel, yaww,
                         path_angle, a, align_ang, theta_d, theta_diff, big_ang, v, last_x_store, last_y_store])
        with open(filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow([x, y, yaw, lookAheadIndex, goal_pose_x, goal_pose_y, vel, ang, ang_vel, yaww,
                                path_angle, a])

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

        last_vel = vel


    with open('after.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csvinput)

    # log_file.close()
    print("kill me")
    sys.stdout.flush()
    # time.sleep(20)
    raw_input("")  # kill 0 sent from bash script not working, so you have to ctrl-c manually

    for process in psutil.process_iter():
        print(process.cmdline())

if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    mu = 0.05
    safety_tol = 4
    env = environments[mu]
    main(mu, safety_tol)

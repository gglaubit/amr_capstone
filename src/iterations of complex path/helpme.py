#!/usr/bin/env python

import sys
import os
import rospy
import datetime
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees
from tf.transformations import euler_from_quaternion
import argparse
import subprocess
from subprocess import Popen
import psutil
import time
import keras
import csv

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0

model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/NNet_all(3).h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)

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
    if run <= 600:
        return 0.009
    elif 600 < run <= 1200:
        return 0.09
    elif 1200 < run <= 1800:
        return 1
    elif 1800 < run <= 2400:
        return 0.05
    elif 2400 < run:
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
    closest_index = dists.index(val)
    return val > safety_tolerance, val, closest_index


def robotAtGoal(robx, roby, goalx, goaly):
    distance_tolerance = 1
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def getLookAheadPoint(waypoints, robx, roby, lookAheadDistance, lastIndex, lastFractionalIndex, lastLookAhead):
    for j in range(lastIndex, len(waypoints) - 1):
        E = waypoints[j]
        L = waypoints[j + 1]
        C = (robx, roby)
        r = lookAheadDistance
        d = (L[0] - E[0], L[1] - E[1])
        f = (E[0] - C[0], E[1] - C[1])
        a = np.dot(d, d)
        b = np.dot(np.multiply(2, f), d)
        c = np.dot(f, f) - r * r
        discriminant = b * b - 4 * a * c

        # this happens on the first waypoint, since the lookahead distance is smaller
        if discriminant < 0:
            return lastLookAhead, lastIndex, lastFractionalIndex

        discriminant = sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        if 0 <= t1 <= 1 and j + t1 > lastFractionalIndex:
            return (E[0] + t1 * d[0], E[1] + t1 * d[1]), j, j + t1
        if 0 <= t2 <= 1 and j + t2 > lastFractionalIndex:
            return (E[0] + t2 * d[0], E[1] + t2 * d[1]), j, j + t2

        # this happens on the last waypoint. I'm not sure if j should be updated on the two
        # return statements above? or if this solution is best - we should figure out why
        # lookahead points aren't working for the first and last waypoint
        return waypoints[lastIndex + 1], j + 1, lastFractionalIndex
    return waypoints[-1], lastIndex, lastFractionalIndex


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
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10*cos(angle), 10*sin(angle))
    print(end_point)
    waypoints = [branching_point, end_point]
    waypoints2 = [(0, 0), branching_point, end_point]
    path = injectPoints(waypoints2)
    lookAheadDistance = 0.5
    lastIndex = 0
    lastFractionalIndex = 0
    lookAheadPoint = waypoints[0]
    atGoalHack = 0  # needs to be fixed
    pred_vels = [0]*4
    begin = datetime.datetime.now()
    time_to_stop = 23  # in minutes

    while not rospy.is_shutdown():
        path = injectPoints(waypoints2)
        unsafe, robot_deviation, closest_index = robotUnsafe(x, y, path, safety_threshold)
        if unsafe:
            print("unsafe")
            stop_robot(vel_msg, velocity_publisher)
            break

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1], 1) and lastIndex == len(path) - 1:
            print("at goal:", x, y)
            stop_robot(vel_msg, velocity_publisher)
            break

        now = datetime.datetime.now()
        # will stop program if robot hasn't found goal or become unsafe after time_to_stop minutes
        if begin + datetime.timedelta(minutes=time_to_stop) < now:
            print("timed out")
            stop_robot(vel_msg, velocity_publisher)
            break

        if robotAtGoal(x, y, goal_pose_x, goal_pose_y, 1):
        lookAheadPoint, lastIndex, lastFractionalIndex = getLookAheadPoint(path, x, y, lookAheadDistance,
                                                                           lastIndex, lastFractionalIndex,
                                                                           lookAheadPoint)

        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]
        #print(lookAheadPoint)
        
        horizon = 0
        while horizon < 4:
            try:
                horizon_point1 = path[closest_index + horizon]
                horizon_point2 = path[closest_index + horizon + 1]
            except IndexError as e:
                horizon_point1 = path[-2]
                horizon_point2 = path[-1]
            a = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0]) - yaw
            ang = abs(degrees(a))
            fut_velocity = model.predict([[mu, ang, safety_threshold]])[0][0]
            pred_vels[horizon] = fut_velocity
            horizon = horizon + 1
        # Current Measure of Safety is slowest but how will that be with more complex systems
        velocity = min(pred_vels)
        
        ang_vel = (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)
        if ang_vel < -(pi/2):
            ang_vel = -pi/2
        elif ang_vel > pi/2:
            ang_vel = pi/2

        # linear velocity in the x-axis:
        vel_msg.linear.x = velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = ang_vel
        
        lookAheadIndex = 0
        a = 0
        with open('helpme_data.csv', 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow([x, y, yaw, lookAheadIndex, goal_pose_x, goal_pose_y, velocity, a, (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)])

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

    #print("Killing now...")
    print("Done.")
    sys.stdout.flush()
    
    os.popen('killall -9 rosmaster')
    os.popen('killall -9 roscore')
    os.popen('killall -9 gzclient')
    os.popen('killall -9 gzserver')


if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)

    #run = rospy.get_param('~run')
    #angle = rospy.get_param('~angle')
    #safety_threshold = rospy.get_param('~safety_threshold')
    #mu = calculate_mu(run)
    #velocity = calculate_velocity(run)
    angle = 110
    mu = 1
    velocity = 1
    safety_threshold = 2
    print("velocity: ", velocity, "angle: ", angle)
    main(velocity, angle, safety_threshold)



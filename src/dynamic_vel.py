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
import keras
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees
from tf.transformations import euler_from_quaternion
import csv

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0
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
                0.7: "ice_7"}

model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/NNet_all_tf_210.h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)


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


def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    # p.wait()  # we wait for children to terminate
    p.terminate()


def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


def terminate_ros_node():
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        # if (str.startswith(s)):
        os.system("rosnode kill " + str)


def robotUnsafe(robx, roby, path, tol):
    safety_tolerance = tol
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        i = i + 1
    val = min(dists)
    closest_index = dists.index(val)
    return val > safety_tolerance, val, closest_index


def robotAtGoal(robx, roby, goalx, goaly, tol):
    distance_tolerance = tol
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def getLookAheadPoint(path, robx, roby, lastLookAhead):
    dx = [robx - pathx[0] for pathx in path]
    dy = [roby - pathy[1] for pathy in path]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d) + 30
    if target_idx <= lastLookAhead:
        target_idx = lastLookAhead + 1
    if target_idx > (len(path) - 1):
        target_idx = (len(path) - 1)
    return target_idx


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


def main(mu, safety_tol):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rospy.Subscriber('/odometry/filtered', Odometry, odomCallback)
    info = "{lin_vel}, {ang_vel}, {angle}, {deviation}\n"
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(45)  # in radians
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10 * cos(angle), 10 * sin(angle))
    #waypoints = [branching_point, end_point]
    #waypoints2 = [(0, 0), branching_point, end_point]
    waypoints = [(20,0), (20, 20), (0, 20), (0,0)]
    waypoints2 = [(0,0), (20,0), (20, 20), (0, 20), (0,0)]
    #waypoints = [(10, 0), (20, 10), (30, 10), (10, 20), (0, 0)]
    #waypoints2 = [(0, 0), (10, 0), (20, 10), (30, 10), (10, 20), (0, 0)]
    space = 0.1
    path = injectPoints(waypoints2, space)
    smooth_path = smoothPath(path)
    atGoalHack = 0
    pred_vels = [0] * 4
    angles = [0] * 4
    filename = "run_data.csv"
    lastLookAhead = 0
    csvinput = []
    test = 0

    while not rospy.is_shutdown():
        path = injectPoints(waypoints2, 0.1)
        unsafe, robot_deviation, closest_index = robotUnsafe(x, y, smooth_path, safety_tol)
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

        target_index = getLookAheadPoint(path, x, y, lastLookAhead)
        lookAheadPoint = path[target_index]
        lastLookAhead = target_index  # lookAheadIndex
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        #print('1')
        horizon = 0
        while horizon < 4:
            try:
                horizon_point1 = path[closest_index + horizon]
                horizon_point2 = path[closest_index + horizon + 1]
            except IndexError as e:
                horizon_point1 = path[-2]
                horizon_point2 = path[-1]
            a = abs(atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])) - abs(yaw)
            ang = abs(degrees(a))
            angles[horizon] = ang
            fut_velocity = 2 #model.predict([[mu, ang, safety_tol]])[0][0]
            pred_vels[horizon] = fut_velocity
            horizon = horizon + 1
        # Current Measure of Safety is slowest but how will that be with more complex systems
        vel = min(pred_vels)
        anglee = angles[pred_vels.index(vel)]

        #print('2')

        theta_d = atan2(goal_pose_y - y, goal_pose_x - x)
        theta_diff = theta_d - yaw
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
        vel_msg.angular.z = 1 * ang_vel

        print(test)
        test = test + 1

        csvinput.append([x, y, yaw, target_index, goal_pose_x, goal_pose_y, vel, anglee, ang_vel, abs(yaw),
                         atan2(goal_pose_y - y, goal_pose_x - x)])
        with open(filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow([x, y, yaw, target_index, goal_pose_x, goal_pose_y, vel, anglee, ang_vel, abs(yaw),
                                atan2(goal_pose_y - y, goal_pose_x - x)])

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

    with open('after.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csvinput)
    # log_file.close()
    print("kill me")
    sys.stdout.flush()
    # time.sleep(20)
    raw_input("")  # kill 0 sent from bash script not working, so you have to ctrl-c manually

    # terminate_process_and_children(proc)
    # signal_process_and_children(proc.pid, signal.SIGINT, True)
    # terminate_ros_node()
    # proc.send_signal(subprocess.signal.SIGINT)
    # proc.kill()
    # proc.terminate()
    for process in psutil.process_iter():
        print(process.cmdline())


if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    mu = 1 #rospy.get_param('~mu')
    safety_tol = 4
    env = environments[mu]
    main(mu, safety_tol)

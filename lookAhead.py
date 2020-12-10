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
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0

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
    'Normalization': keras.layers.experimental.preprocessing.Normalization()},compile=False)

def statesCallback(data):
    global x, y, v, yaw
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


def robotUnsafe(robx, roby, path):
    # print("hi")
    safety_tolerance = 2
    dists = [0] * len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
        # print(i)
        i = i + 1
    # print(dists),
    val = min(dists)
    closest_index = dists.index(val)
    # print(val)
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
        # print('t guys', t1, t2)
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
            # print(new_list)
            new_points.append(new_list)
        new_points.append(end_point)
    # print(new_points)
    return new_points


def smoothPath(path):  # path is [(x1, y1), ..., (xend, yend)]
    b = 0.75
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


def vel_pid(vg, vc, dt):
    v_goal = vg # output of neural net in m/s
    v_curr = vc # current velocity in m/s
    prev_err = 0.0
    windup_guard = 10 # needs to be changed??
    kp = 1
    ki = 0.1
    kd = 0.1
    error = v_goal - v_curr
    delta_error = error - prev_err
    p = kp*error
    i = i + error * dt
    if i < -windup_guard:
        i = windup_guard
    elif i > windup_guard:
        i = windup_guard
    d = 0.0 # use acceleration calculated above??
    if delta_time > 0:
        d = delta_error/dt
    prev_err = error
    vel = p + ki*i + kd*d
    return vel


# def main(velocity, angle_deg, log_file, run_num):
def main(velocity, angle_deg, run_num):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    # velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    info = "{lin_vel}, {ang_vel}, {angle}, {deviation}\n"
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(angle_deg)  # in radians
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10 * cos(angle), 10 * sin(angle))
    print(end_point)
    # waypoints = [(10, 0), (0, 10), (10, 10), (0, 0)]
    waypoints = [branching_point, end_point]
    waypoints2 = [(0, 0), branching_point, end_point]
    path = injectPoints(waypoints2)
    lookAheadDistance = 10
    horizon = 10
    lastIndex = 0
    # lastLookAheadIndex = 0
    lastFractionalIndex = 0
    lookAheadPoint = waypoints[0]
    atGoalHack = 0  # needs to be fixed
    start_angle = yaw
    # i = 0
    fut_velocity = 0.5

    # bag_location = "bagfiles/{env}/trainingData".format(env=environments[mu]) + run_num
    # command = "rosbag record -O " + bag_location + " /gazebo/model_states /odometry/filtered /imu/data"
    # proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')

    while not rospy.is_shutdown():
        path = injectPoints(waypoints2)
        unsafe, robot_deviation, closest_index = robotUnsafe(x, y, path)
        if unsafe:
            print("unsafe")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            # log_file.write(info.format(lin_vel=0, ang_vel=0, angle=angle_deg,
            #                           deviation=robot_deviation))
            break
        # print(unsafe)
        # print(robot_deviation)

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastIndex == len(waypoints) - 1:
            # if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastIndex == len(waypoints) - 1 and atGoalHack>100:
            print("at goal:", x, y)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            # log_file.write(info.format(lin_vel=0, ang_vel=0, angle=angle_deg,
            #                           deviation=robot_deviation))
            break

        lookAheadPoint, lastIndex, lastFractionalIndex = getLookAheadPoint(waypoints, x, y, lookAheadDistance,
                                                                           lastIndex, lastFractionalIndex,
                                                                           lookAheadPoint)
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        # mu = mu[horizon] ## This is just a general concept for when we don't have a constant mu
        horizon_point1 = path[closest_index + horizon]
        horizon_point2 = path[closest_index + horizon + 1]
        # Estimate angle from starting pose
        angl = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0]) - start_angle
        # estimate angle from current pose
        #angl = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0]) - yaw
        fut_velocity = model.predict([[mu, angl]])[0][0]
        # linear velocity in the x-axis:
        vel_msg.linear.x = fut_velocity
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 2 * (atan2(goal_pose_y - y, goal_pose_x - x) - yaw)

        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1

        # writing to the log file
        # log_file.write(info.format(lin_vel=vel_msg.linear.x, ang_vel=vel_msg.angular.z,
        #                           angle=angle_deg, deviation=robot_deviation))

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

    # get run number
    run = rospy.get_param('~run')
    angle = rospy.get_param('~angle')
    run_num = str(run)

    # automatically calculate angle
    # automatically calculate mu
    mu = 0
    if run <= 120:
        mu = 0.009
    elif 120 < run <= 240:
        mu = 0.09
    elif 240 < run <= 360:
        mu = 1
    elif 360 < run <= 480:
        mu = 0.05
    elif 480 < run <= 600:
        mu = 0.5
    elif 600 < run <= 720:
        mu = 0.02
    elif 720 < run <= 840:
        mu = 0.2
    elif 840 < run <= 960:
        mu = 0.07
    elif 960 < run:
        mu = 0.7

    # use neural network to choose velocity

    # velocity = model.predict([[mu, angle]])[0][0]
    # velocity = 1.0
    env = environments[mu]

    # bag_location = "bagfiles/trainingData" + args.run_num

    # log_file = "../logs/{run}_{env}_{vel}_{angle}.txt".format(run=run_num, env=env, vel=str(velocity), angle=str(angle)) # this needs to be fixed, right now you run test.sh from the bagfiles directory
    # file format: velocity, angle, path_deviation
    # file = open(log_file, "w")

    print("velocity: ", velocity, "angle: ", angle)
    # print(angle, args.angle)
    # main(velocity, angle, file, run_num)
    main(velocity, angle, run_num)

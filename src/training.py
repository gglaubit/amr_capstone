#!/usr/bin/env python

import rospy
import argparse
import subprocess
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians
from tf.transformations import euler_from_quaternion

x = 0.0
y = 0.0
v = 0.0
yaw = 0.0


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


def robotUnsafe(robx, roby, path):
    safety_tolerance = 10
    dists = [0]*len(path)
    i = 0
    for point in path:
        dists[i] = sqrt(pow((point[0] - robx), 2) + pow((point[1] - roby), 2))
    i += 1
    val = min(dists)
    return val > safety_tolerance, val


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
    spacing = 5
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


def main(velocity, angle_deg, log_file):
    rospy.init_node('pure_pursuit_cap', anonymous=True)
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    # velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    info = "{lin_vel}, {ang_vel} {angle}, {mu}, {deviation}"
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(angle_deg)  # in radians
    branching_point = (10, 0)
    end_point = (branching_point[0] + 10*cos(angle), 10*sin(angle))
    print(end_point)
    # waypoints = [(10, 0), (0, 10), (10, 10), (0, 0)]
    waypoints = [branching_point, end_point]
    path = injectPoints(waypoints)
    lookAheadDistance = 2
    lastIndex = 0
    # lastLookAheadIndex = 0
    lastFractionalIndex = 0
    lookAheadPoint = waypoints[0]
    atGoalHack = 0  # needs to be fixed
    # i = 0

    while not rospy.is_shutdown():
        unsafe, robot_deviation = robotUnsafe(x, y, path)
        if unsafe:
            print("unsafe")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            log_file.write(info.format(lin_vel=0, ang_vel=0, angle=angle_deg,
                                       mu="???", deviation=robot_deviation))
            break

        if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastIndex == len(waypoints) - 1:
        #if robotAtGoal(x, y, waypoints[-1][0], waypoints[-1][1]) and lastIndex == len(waypoints) - 1 and atGoalHack>100:
            print("at goal")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            log_file.write(info.format(lin_vel=0, ang_vel=0, angle=angle_deg,
                                       mu="???", deviation=robot_deviation))
            break

        lookAheadPoint, lastIndex, lastFractionalIndex = getLookAheadPoint(waypoints, x, y, lookAheadDistance,
                                                                           lastIndex, lastFractionalIndex,
                                                                           lookAheadPoint)
        goal_pose_x = lookAheadPoint[0]
        goal_pose_y = lookAheadPoint[1]

        # linear velocity in the x-axis:
        vel_msg.linear.x = velocity
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
        log_file.write(info.format(lin_vel=vel_msg.linear.x, ang_vel=vel_msg.angular.z,
                                   angle=angle_deg, mu="???", deviation=robot_deviation))
    # process.kill()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Set the velocity of the robot.')
    # parser.add_argument("--velocity", type=float, help='velocity of the robot', default=0.0)
    parser.add_argument("--angle", type=float, help='angle of the robot in degrees', default=0.0)
    parser.add_argument("--run_num", help='run of the robot', default=0)
    
    args = parser.parse_args()
    
    bag_location = "bagfiles/trainingData" + args.run_num
    log_file = "training_log" + args.run_num + ".txt"

    # file format: velocity, angle, mu, path_deviation
    file = open(log_file, "w")

    # process = subprocess.Popen(["rosbag", "record", "-O", bag_location, "/gazebo/model_states", "/odometry/filtered"])

    velocity = 0.0
    # comp_angle = 0

    run_num = int(args.run_num)
    if run_num % 3 == 0:
        velocity = 2.0
    elif run_num % 3 == 1:
        velocity = 0.2
    elif run_num % 3 == 2:
        velocity = 1.0

    '''
    run_list = [1, 2, 3, 31, 32, 33, 61, 62, 63]
    if run_num in run_list:
        comp_angle = 0
    elif run_num in [x+3 for x in run_list]:
        comp_angle = 20
    elif run_num in [x+6 for x in run_list]:
        comp_angle = 40
    elif run_num in [x+9 for x in run_list]:
        comp_angle = 60
    elif run_num in [x+12 for x in run_list]:
        comp_angle = 80
    elif run_num in [x+15 for x in run_list]:
        comp_angle = 100
    elif run_num in [x+18 for x in run_list]:
        comp_angle = 120
    elif run_num in [x+21 for x in run_list]:
        comp_angle = 140
    elif run_num in [x+24 for x in run_list]:
        comp_angle = 160
    elif run_num in [x+27 for x in run_list]:
        comp_angle = 180
    '''

    print("velocity: ", velocity, '\n', "angle: ", args.angle)
    main(velocity, int(args.angle), file)
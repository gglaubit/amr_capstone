#!/usr/bin/env python

import sys  # for redirecting output in bash, could be removed
#import time  # for sleeping - time.sleep is commented out below right now
from time import clock #perf_counter
import os
import rospy
import argparse
import subprocess
from subprocess import Popen
import psutil
from geometry_msgs.msg import Twist
import numpy as np
import keras
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees, exp, log
from tf.transformations import euler_from_quaternion
import csv

x = 0.0
y = 0.0
v = 0.0
v_odom = 0.0
yaw_gazebo = 0.0
RAMP_UP = False
RAMP_DOWN = False
REPLAN1 = 0
REPLAN2 = 0
MIN_VEL = 0.1
MAX_VEL = 2.0
first_run = 1
index = 0
ramp_ang = 0
ramp_len = 0

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

#surface_model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/2_speed_network(feb1).h5', custom_objects={
#    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)
    
surface_model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/gaussian_prediction_network_with_noise.h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)

success_model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/ramp_success_network_MAR1.h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)

ramp_model = keras.models.load_model('/home/bezzo/catkin_ws/src/capstone_nodes/1_speed_ramp_network_MAR1.h5', custom_objects={
    'Normalization': keras.layers.experimental.preprocessing.Normalization()}, compile=False)


def statesCallback(data):
    global x, y, v, yaw_gazebo, quat, first_run, index
    if first_run:
        name = data.name
        index = name.index("jackal")
        first_run = 0
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
    yaw_gazebo = euler[2]
    

def odomCallback(data):
    global v_odom
    v_odom = data.twist.twist.linear.x
    
def laserscanCallback(msg):
	# values at 0 degree
	msg.ranges[360]

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

def determineRamp(robx, roby,  tol):
	global ramp_ang, ramp_len, RAMP_UP, RAMP_DOWN, REPLAN1, REPLAN2
	
	if abs(robx-56.8) < tol and roby > 40 and roby < 55:
		#print("First up ramp")
		REPLAN2 = REPLAN2 + 1
		RAMP_UP = True
		ramp_ang = 0.4
		ramp_len = 3
	elif abs(robx-30) < tol and roby > 40 and roby < 55:
		#print("Second up ramp")
		REPLAN1 = REPLAN1 + 1
		RAMP_UP = True
		ramp_ang = 0.6
		ramp_len = 1.5
	elif abs(robx-20) < tol and roby > 40 and roby < 55:
		#print("First down ramp")
		RAMP_DOWN = True
		RAMP_UP = False
		ramp_ang = -0.6
		ramp_len = 1
	elif abs(robx-10) < tol and roby > 40 and roby < 55:
		#print("Second down ramp")
		RAMP_DOWN = True
		ramp_ang = -0.4
		ramp_len = 1
	elif abs(robx-1) < tol:
		RAMP_DOWN = False
		
		
	

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


def checkAlignment(closest_index, horizon, x, y, last_x, last_y, path_predict, tol):
    try:
        horizon_point1 = path_predict[closest_index + horizon]
        horizon_point2 = path_predict[closest_index + horizon + 1]
    except IndexError as e:
        horizon_point1 = path_predict[-2]
        horizon_point2 = path_predict[-1]
    path_angle = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])
    yaw_calc = atan2(y - last_y, x - last_x)
    a = path_angle - yaw_calc
    ang = abs(degrees(a))
    return ang - tol < 0, ang


def calculate_lookahead(mu):
    if mu >= 0.2:
        return 5
    elif 0.2 < mu <= 0.04:
        return 10
    elif 0.04 < mu < 0.009:
        return 20
    else:
        return 30
        

def calculate_checkcount(mu):
    if mu >= 0.05:
        return 5
    elif 0.05 < mu < 0.009:
        return 10 
    else:
        return 20

def find_ramp_velocity(mu, length, ang): 
	result = 0.0
	pred = success_model.predict([[mu,length,ang]]) [0][0]
	print(pred)
	if pred > 0.5: 
		result = ramp_model.predict([[mu,length,ang]])[0][0]
		print(result)
		result = 0.5
	return result

def main(mu, safety_tol):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rospy.Subscriber('/odometry/filtered', Odometry, odomCallback) 
    rospy.Subscriber('/front/scan', LaserScan, laserscanCallback)       
    rate = rospy.Rate(10)
    vel_msg = Twist()
    angle = radians(45)  # in radians
    branching_point = (15, 0)
    end_point = (branching_point[0] + 15 * cos(angle), 15 * sin(angle))
    # waypoints = [branching_point, end_point]
    # waypoints2 = [(0, 0), branching_point, end_point]
    
    #waypoints = [branching_point, end_point]
    #waypoints2 = [(0, 0), branching_point, end_point]
    
    # squares
    #waypoints = [(20,0), (20, 20), (0, 20), (0,0)] # counter clockwise
    #waypoints2 = [(0,0), (20,0), (20, 20), (0, 20), (0,0)]
    #waypoints = [(0,20), (20, 20), (20, 0), (0,0)]  # clockwise
    #waypoints2 = [(0,0), (0,20), (20, 20), (20, 0), (0,0)]
    
    # mixed
    #waypoints = [(10,20), (30,10), (20,10), (10,0), (1,0)] # reverse
    #waypoints2 = [(0,0), (10,20), (30,10), (20,10), (10,0), (1,0)]
    #waypoints = [(10,0), (20, 10), (30, 10), (10, 20), (-20,20)]   
    #waypoints2 = [(0,0), (10,0), (20, 10), (30, 10), (10, 20), (-20,20)]
    
    # test_world 3m
    #waypoints = [(21.5,-1.7), (46,22.675),(46, -1.7), (65, -1.7),(65, 33.5),(55.9,49.3),(-14, 49.3),(-14, -1.7)]
    #waypoints2 = [(0,-1.7), (21.5,-1.7), (46,22.675),(46, -1.7), (65, -1.7),(65, 33.5),(55.9,49.3),(-14, 49.3),(-14, -1.7)]
    
    # test_world 1m    
    waypointss1 = [(22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(-16,51.3),(-16,-3.7)]
    waypointss1b = [(0,-3.7), (22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(-16,51.3),(-16,-3.7)]
    
    waypointss2 = [(22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(25,51.3),(25,31.3),(11,31.3),(11,51.3),(-16,51.3),(-16,-3.7)]
    waypointss2b =[(0,-3.7), (22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(25,51.3),(25,31.3),(11,31.3),(11,51.3),(-16,51.3),(-16,-3.7)]
    
    waypointss3 = [(22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(44,51.3),(44,59),(-16,59),(-16,-3.7)]
    waypointss3b = [(0,-3.7), (22.5,-3.7),(44,17.675),(44,-3.7), (67,-3.7),(67,34.75),(57.5,51.3),(44,51.3),(44,59),(-16,59),(-16,-3.7)]
   
    obstacle1 = 0
    obstacle2 = 0
    path_choices = [[1, waypointss1, waypointss1b], [2, waypointss2, waypointss2b], [3, waypointss3, waypointss3b]]

    waypoints = path_choices[0][1]
    waypoints2 = path_choices[0][2]
    
    space = 0.1
    path = injectPoints(waypoints2, space)
    path_predict = injectPoints(waypoints2, 0.5)
    smooth_path = smoothPath(path)
    lookahead_num = calculate_lookahead(mu)
    stored_angs = [0] * lookahead_num
    lastLookAhead = 0
    lookAheadIndex = getLookAheadPoint(path, x, y, lastLookAhead)
    lookAheadPoint = path[lookAheadIndex]
    goal_pose_x = lookAheadPoint[0]
    goal_pose_y = lookAheadPoint[1]
    aligned = 0
    check_count = calculate_checkcount(mu)
    lastLookAhead = lookAheadIndex
    last_x = 0
    last_y = 0
    last_vel = 0
    csvinput = []
    csvinput2 = []
    filename = "run_data_dynamic_help.csv"
    print("lookahead:", lookahead_num, "angle:", degrees(angle), "mu", mu)
    print(safety_tol)
    start = clock() #perf_counter()

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
	
        determineRamp(x, y, safety_tol)
        	
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
            path_angle = atan2(horizon_point2[1] - horizon_point1[1], horizon_point2[0] - horizon_point1[0])
            yaw_calc = atan2(y - last_y, x - last_x)
            a = path_angle - yaw_calc
            
            if abs(a) > pi:
            	a = abs(a) - 2*pi
            
            ang = abs(degrees(a))
            stored_angs[horizon] = ang
            horizon = horizon + 1
        
        d = sqrt(pow((x - last_x), 2) + pow((y - last_y), 2))
        t = clock() - start
        calc_v = d/t
        start = clock()
        last_x = x
        last_y = y

        #big_ang = max(stored_angs)
        #velo_ = surface_model.predict([[mu, big_ang, safety_tol]])[0][1]
        #velo = max(velo_, 0.05)
        
        big_ang = max(stored_angs)
        runs = 10
        inputt = [mu, big_ang, safety_tol]
        point_predictions = surface_model.predict([inputt for run in range(0, runs)])[...,1] # faster than np.tile for small arrays
        point_predictions = np.array(point_predictions)
        variance = point_predictions.var(axis = 0, ddof = 1) # unbiased estimator
        mean = point_predictions.mean()
        predicted_speed = (mean - 2 * (variance**0.5))
        velo = max(predicted_speed, MIN_VEL)
        
        isAligned, align_ang = checkAlignment(closest_index, 6, x, y, last_x, last_y, path_predict, 15)
        
        if isAligned:
            aligned += 1
        else:
            aligned = 0
        
        if (aligned > check_count or velo < last_vel):
            vel = velo
        else:
            vel = max(last_vel, MIN_VEL)
        
        #print(vel, big_ang, closest_index)
        


        if (RAMP_UP == True) or (RAMP_DOWN == True):
            #print('ramp!')
            vel_bound = find_ramp_velocity(mu, ramp_len, ramp_ang)
            if vel_bound == 0:
                #replan
                if REPLAN1 == 1:
                    print('replan!')
                    waypoints = path_choices[1][1]
                    waypoints2 = path_choices[1][2]
                    path = injectPoints(waypoints2, space)
                    path_predict = injectPoints(waypoints2, 0.5)
                    smooth_path = smoothPath(path)
                elif REPLAN2 == 1:
                    print('replan 2!')
                    waypoints = path_choices[2][1]
                    waypoints2 = path_choices[2][2]
                    path = injectPoints(waypoints2, space)
                    path_predict = injectPoints(waypoints2, 0.5)
                    smooth_path = smoothPath(path)
                    
            elif (RAMP_UP == True):
                if vel < vel_bound:
		            vel = vel_bound
            elif (RAMP_DOWN == True):
                if vel > vel_bound:
                    vel = vel_bound
        

        theta_d = atan2(goal_pose_y - y, goal_pose_x - x)
        theta_diff = theta_d - yaw_gazebo
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
        vel_msg.angular.z = ang_vel * 4

        csvinput.append([x, y, yaw_calc, lookAheadIndex, goal_pose_x, goal_pose_y, vel, ang, ang_vel, yaw_gazebo,
                         path_angle, a, align_ang, theta_d, theta_diff, big_ang, v, last_x, last_y])

        csvinput2.append([x, y, vel, v, v_odom])
        
        with open(filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            csvwriter.writerow([x, y, yaw_calc, lookAheadIndex, goal_pose_x, goal_pose_y, vel, ang, ang_vel, yaw_gazebo,
                                path_angle, a, closest_index])


        # publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        rate.sleep()

        last_vel = vel


    with open('after.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csvinput)
        
    with open('vel_analysis.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerows(csvinput2)

    # log_file.close()
    print("kill me")
    sys.stdout.flush()
    # time.sleep(20)
    raw_input("")  # kill 0 sent from bash script not working, so you have to ctrl-c manually

    #for process in psutil.process_iter():
    #    print(process.cmdline())

if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    mu = 1
    safety_tol = 1
    env = environments[mu]
    main(mu, safety_tol)

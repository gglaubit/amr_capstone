#!/usr/bin/env python

import sys  # for redirecting output in bash, could be removed
import time # for sleeping - time.sleep is commented out below right now
import os
import rospy
import argparse
import subprocess
from subprocess import Popen
import psutil
import time
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from math import pow, atan2, sqrt, ceil, sin, cos, pi, radians, degrees
import csv
from std_srvs.srv import Empty


x = 0.0
y = 0.0
v = 0.0


def statesCallback(data):
    global x, y, v, yaw, quat
    # find index of slash
    name = data.name
    index = name.index("jackal")
    # index = name.index("/")
    x = data.pose[index].position.x
    y = data.pose[index].position.y
    v = data.twist[index].linear.x


def robotAtGoal(robx, roby, goalx, goaly, tol):
    distance_tolerance = tol
    val = sqrt(pow((goalx - robx), 2) + pow((goaly - roby), 2))
    return val <= distance_tolerance


def main(vel, mu):
    velocity_publisher = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, statesCallback)
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    info = "{lin_vel}, {ang_vel}, {angle}, {deviation}\n"
    rate = rospy.Rate(10)
    vel_msg = Twist()
    atGoalHack = 0
    goal_x = 15
    goal_y = 0
    stopped = 0
    filename = "stopping_dist_data.csv"
    csvinput = []
    velocity = vel

    while not rospy.is_shutdown():
        
        if robotAtGoal(x, y, goal_x, goal_y, 0.5):
            stopped = 1
            start_x = x
            start_y = y
            start_time = time.time()
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
            vel = 0

        if v < 0.0001 and stopped == 1:
            distance = sqrt(pow((x - start_x), 2) + pow((y - start_y), 2))
            end_time = time.time()
            dt = end_time - start_time
            break

        # linear velocity in the x-axis:
        vel_msg.linear.x = vel
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # angular velocity in the z-axis:
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        velocity_publisher.publish(vel_msg)
        rate.sleep()
        atGoalHack += 1
        
    with open(filename, 'a') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        csvwriter.writerow([mu, velocity, distance, dt])

    #print("kill me")
    #sys.stdout.flush()
    #raw_input("")  # kill 0 sent from bash script not working, so you have to ctrl-c manually

    #for process in psutil.process_iter():
    #    print(process.cmdline())


if __name__ == "__main__":
    rospy.init_node('capstone_nodes', anonymous=True)
    mu = 0.5
    vel = 0.1
    main(vel, mu)

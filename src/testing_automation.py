#!/usr/bin/env python

import subprocess
import numpy as np
from collections import OrderedDict
import os
import signal
import time

import rosbag
from std_msgs.msg import Int32, String
import psutil


environments = OrderedDict([("ice_env_009.launch", 0.009),
                            ("ice_env_09.launch", 0.09),
                            ("ice_env_9.launch", 0.9), 
                            ("control_env.launch", 1), 
                            ("mud_env.launch", 1000)])

environment = {"mud_env.launch": 1000}
angle_step = 10  # in degrees
velocity_step = 0.2
test_num = 761

# required so that a bash shell is used rather than sh, so we can use Gazebo aliases in ~/.bashrc
# jk used this will shell=True in the subprocess command and that was not good idk why
cmd = ["/bin/bash", "-i", "-c"] 


def terminate_process_and_children(p):
    ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % p.pid, shell=True, stdout=subprocess.PIPE)
    ps_output = ps_command.stdout.read()
    retcode = ps_command.wait()
    assert retcode == 0, "ps command returned %d" % retcode
    for pid_str in ps_output.split("\n")[:-1]:
            os.kill(int(pid_str), signal.SIGINT)
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
        #if (str.startswith(s)):
        os.system("rosnode kill " + str)


for env in environment.keys():
    for angle in list(np.arange(0, 190, angle_step)):    
        for vel in list(np.arange(0.2, 2.2, velocity_step)):
            # start gazebo
            print("gazebo")
            # gazebo = subprocess.Popen(cmd + ["roslaunch", "cpr_agriculture_gazebo", env, "gui:=false", shell=True)
            gazebo = subprocess.Popen(["roslaunch", "cpr_agriculture_gazebo", env, "gui:=false"], 
                                     cwd=os.path.expanduser("~/catkin_ws/src/cpr_gazebo/cpr_agriculture_gazebo"))
            #gazebo = subprocess.call(["roslaunch cpr_agriculture_gazebo " + env + " gui:=false"], 
            #                         cwd=os.path.expanduser("~/catkin_ws"), shell=True, stdin=subprocess.PIPE)
            time.sleep(3)
            
            # start recording the rosbag
            # print("start rosbag")
            bag_location = "bagfiles/{env}/trainingData".format(env=env) + str(test_num)
            print(bag_location)
            #rosbag = subprocess.Popen(["rosbag", "record", "-O", bag_location, "/gazebo/model_states", 
            #                          "/odometry/filtered"])
            rosbag = subprocess.Popen("rosbag record -O " + bag_location + " /gazebo/model_states /odometry/filtered",
                                       stdin=subprocess.PIPE, shell=True)
            time.sleep(3)
            
            # run python script
            print("python script")
            pure_pursuit = subprocess.Popen(["rosrun", "capstone_nodes", "python", "training.py", 
                                             "--v={vel}".format(vel=vel), 
                                             "--a={angle}".format(angle=angle), 
                                             "--m={mu}".format(mu=environments[env]), 
                                             "--r={test_num}".format(test_num=test_num)])
            pure_pursuit.wait()

            #pure_pursuit = subprocess.Popen("python training.py --v={vel} --a={angle} --m={mu} --r={test_num}".format(
            #                                vel=vel, angle=angle, mu=environments[env], test_num=test_num), 
            #                                shell=True).wait()


            # pure_pursuit.send_signal(subprocess.signal.SIGINT)
            
            rosbag.send_signal(subprocess.signal.SIGINT)
            # rosbag.kill()
            #signal_process_and_children(rosbag.pid, subprocess.signal.SIGINT)
            #terminate_process_and_children(rosbag)
            # terminate_ros_node()
            #time.sleep(2)

            # terminate_process_and_children(gazebo)
            #gazebo.send_signal(subprocess.signal.SIGINT)
            gazebo.kill()
            
            test_num += 1



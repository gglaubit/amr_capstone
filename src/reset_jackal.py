#!/usr/bin/env python

import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def main():
    rospy.init_node('set_pose')
    state_msg0 = ModelState()
    state_msg0.model_name = 'jackal'
    state_msg0.pose.position.x = 0
    state_msg0.pose.position.y = 0
    state_msg0.pose.position.z = 0
    state_msg0.pose.orientation.x = 0
    state_msg0.pose.orientation.y = 0
    state_msg0.pose.orientation.z = 0
    state_msg0.pose.orientation.w = 0
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp0 = set_state( state_msg0 )

    except rospy.ServiceException, e:
	    print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

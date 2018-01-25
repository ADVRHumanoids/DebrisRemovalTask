#!/usr/bin/env python

"""
This script pub a valve pose topic wrt world odom (feet center of walkman robot)

usage:
./pubValvePoseWrtWorldOdom.py
"""


# this program is used to publish the pose of valve, walkman in gound frame and also valve wrt walkman

import sys
import rospy
from gazebo_ros.gazebo_interface import *


def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def usage():
    return "%s [model_name] [relative_entity_name]"%sys.argv[0]


if __name__ == "__main__":

    valve_pose_msg = gms_client('valve','world')
    walkman_pose_msg = gms_client('walkman','world')

    print("valve_pose_msg", valve_pose_msg)
    print("walkman_pose_msg", walkman_pose_msg)
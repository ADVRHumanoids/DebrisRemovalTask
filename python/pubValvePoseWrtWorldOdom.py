#!/usr/bin/env python

"""
This script pub a valve pose topic wrt world odom (feet center of walkman robot)

usage:

./pubValvePoseWrtWorldOdom.py

"""


import tf
import sys
import time
import rospy
from gazebo_ros.gazebo_interface import *
import tf.transformations as trans

import numpy as np
import geometry_msgs.msg

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

def poseToMatrix(pose):
    position = np.array([pose.position.x,
                         pose.position.y,
                         pose.position.z])
    translation_matrix = trans.translation_matrix(position)
    orientation = np.array([pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w])
    rotation_matrix = trans.quaternion_matrix(orientation)
    transformation_matrix = translation_matrix.dot(rotation_matrix)
    return transformation_matrix

def matrixToPose(Trans):
    position = Trans[0:3,3]
    orientation = trans.quaternion_from_matrix(Trans)

    pose = geometry_msgs.msg.Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

def getValveHandlePoseWrtWorldOdom():
    # get valve ground pose
    valve_pose_msg = gms_client('valve', 'world')
    if valve_pose_msg.success is True:
        print("Got valve pose!")
        ground_Trans_valve = poseToMatrix(valve_pose_msg.pose)
        print(ground_Trans_valve[:3,3])
    else:
        print("Fail to get valve pose!")

    # get walkman ground pose
    walkman_pose_msg = gms_client('walkman', 'world')
    if walkman_pose_msg.success is True:
        print("Got walkman pose!")
        ground_Trans_walkman = poseToMatrix(walkman_pose_msg.pose)
        print(ground_Trans_walkman[:3, 3])
    else:
        print("Fail to get walkman pose!")

    # define world_odom
    ground_Trans_worldOdom = ground_Trans_walkman
    ground_Trans_worldOdom[2, 3] = 0.0

    # calculate valve wrt worldOdom
    worldOdom_Trans_valve = np.linalg.inv(ground_Trans_worldOdom).dot(ground_Trans_valve)

    # define valve handle wrt valve model base
    # valve_Translation_handle = trans.translation_matrix(np.array([0.2, 0.0, 1.2]))

    walkman_palm_length = 0.1
    handle_shift_up = 0.2
    valve_Translation_handle = trans.translation_matrix(np.array([0.2+walkman_palm_length, 0.0, 1.2+handle_shift_up]))
    valve_Rotation_handle = trans.euler_matrix(0.0,-np.pi/2,np.pi,'sxyz')
    valve_Trans_handle = valve_Translation_handle.dot(valve_Rotation_handle)


    # calculate valve handle wrt worldOdom
    worldOdom_Trans_handle = worldOdom_Trans_valve.dot(valve_Trans_handle)
    print("worldOdom_Trans_handle:", worldOdom_Trans_handle[:3, 3])



    # prepare worldOdom_Trans_handle msg
    worldOdom_pose_handle = matrixToPose(worldOdom_Trans_handle)

    return worldOdom_pose_handle


if __name__ == "__main__":





    rospy.init_node('valve_pose_pub_node')
    pub = rospy.Publisher('valve_pose', geometry_msgs.msg.PoseStamped, queue_size=1)



    time.sleep(0.5)



    msg = geometry_msgs.msg.PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world_odom'
    msg.pose = getValveHandlePoseWrtWorldOdom()
    pub.publish(msg)









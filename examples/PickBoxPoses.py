# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:35:28 2018

@author: arturo
"""

import geometry_msgs.msg as geomsg

def get_down_open_lr():
    
    l_pose = geomsg.Pose()
    l_pose.position.x = 0.20
    l_pose.position.y = 0.35
    l_pose.position.z = -0.6
    l_pose.orientation.x = 0.0
    l_pose.orientation.y = -0.4
    l_pose.orientation.z = 0.0
    l_pose.orientation.w = 0.9
    
    r_pose = geomsg.Pose()
    r_pose.position.x = 0.20
    r_pose.position.y = -0.35
    r_pose.position.z = -0.6
    r_pose.orientation.x = 0.0
    r_pose.orientation.y = -0.4
    r_pose.orientation.z = 0.0
    r_pose.orientation.w = 0.9
    
    return (l_pose, r_pose)
    
def get_down_close_lr():
    
    l_pose = geomsg.Pose()
    l_pose.position.x = 0.20
    l_pose.position.y = 0.25
    l_pose.position.z = -0.6
    l_pose.orientation.x = 0.0
    l_pose.orientation.y = -0.4
    l_pose.orientation.z = 0.0
    l_pose.orientation.w = 0.9
    
    r_pose = geomsg.Pose()
    r_pose.position.x = 0.20
    r_pose.position.y = -0.25
    r_pose.position.z = -0.6
    r_pose.orientation.x = 0.0
    r_pose.orientation.y = -0.4
    r_pose.orientation.z = 0.0
    r_pose.orientation.w = 0.9
    
    return (l_pose, r_pose)
    
def get_up_close_lr():
    
    l_pose = geomsg.Pose()
    l_pose.position.x = 0.20
    l_pose.position.y = 0.25
    l_pose.position.z = -0.0
    l_pose.orientation.x = 0.0
    l_pose.orientation.y = -0.70
    l_pose.orientation.z = 0.0
    l_pose.orientation.w = 0.70
    
    r_pose = geomsg.Pose()
    r_pose.position.x = 0.20
    r_pose.position.y = -0.25
    r_pose.position.z = -0.0
    r_pose.orientation.x = 0.0
    r_pose.orientation.y = -0.70
    r_pose.orientation.z = 0.0
    r_pose.orientation.w = 0.70
    
    return (l_pose, r_pose)
    
def get_pass_box_torso():
    
    pose = geomsg.Pose()
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.70
    pose.orientation.w = 1.0
    
    return pose
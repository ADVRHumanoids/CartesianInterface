#!/usr/bin/env python


# -*- coding: utf-8 -*-
"""
Created on Mon Nov 19 17:50:12 2018

@author: arturo.laurenzi@iit.it
"""

from cartesian_interface import roscpp_utils
from cartesian_interface import pyci_ros
import sys
import rospy
import numpy
import math
import timeit

if __name__ == '__main__':
    
    # Initialize both rospy and roscpp 
    ### (remember that CartesianInterfaceRos is actually implemented in C++!)
    rospy.init_node('example_node', sys.argv)
    roscpp_utils.init('example_node', sys.argv)
    
    # Get a CartesI/O ROS implementation
    ci = pyci_ros.CartesianInterfaceRos()
    
    # Query list of defined tasks
    task_list = ci.getTaskList()
    
    print 'Task list is: ', task_list
    
    # Select a task
    task_name = task_list[0]
    
    # Query base link and control mode of a task
    print 'Base link of task ', task_name, ' is ', ci.getBaseLink(task_name)
    print 'Control mode of task ', task_name, ' is ', ci.getControlMode(task_name)

    # Read current reference for a task 
    pos, rot, _, _ = ci.getPoseReference(task_name)
    
    print 'Pose reference of task ', task_name, ' is: '
    print '    pos: ', pos
    print '    rot: ', rot
    
    # Define a trajectory with three waypoints
    # Use the class WayPoint to define them
    pos[0] += 0.3
    w1 = pyci_ros.WayPoint()
    w1.position = pos.copy()
    w1.orientation = rot.copy()
    w1.time = 2.0
    
    pos[2] += 0.3
    w2 = pyci_ros.WayPoint()
    w2.position = pos.copy()
    w2.orientation = rot.copy()
    w2.time = 4.0
    
    pos[0] -= 0.3
    pos[2] -= 0.3
    w3 = pyci_ros.WayPoint()
    w3.position = pos.copy()
    w3.orientation = rot.copy()
    w3.time = 4.1
    
    print 'Commanding reach motion with waypoints'
    print w1
    print w2
    print w3
    
    # Send trajectory
    ci.setWaypoints(task_name, [w1, w2, w3], False)
    
    # Compute the time required to execute the trajectory
    tic = timeit.default_timer()
    ci.waitReachCompleted(task_name)
    toc = timeit.default_timer()
    
    print 'Reach completed in ', toc-tic
    print 'Expected time was ', w3.time
    print 'Possible difference is due to velocity/acceleration limits!'
    
    # Change posture reference
    joint_ref = {'knee_pitch_1':  1.5, 
                 'knee_pitch_2': -1.5}
    
    ci.setReferencePosture(joint_ref)
    
    rospy.sleep(rospy.Duration(3.0))
    
    joint_ref = {'knee_pitch_1': -1.5, 
                 'knee_pitch_2': +1.5}
    
    ci.setReferencePosture(joint_ref)
    
    # Send an online reference (sinusoidal) @30 Hz
    freq = 30.0
    rate = rospy.Rate(freq)
    ampl = 0.10
    T = 4.0
    t = 0.0
    
    while not rospy.is_shutdown():
        ref = pos.copy()
        ref[0] += ampl * math.sin(2*math.pi*t/T)
        ci.setPoseReference(task_name, ref, rot)
        t += 1./freq
        rate.sleep()
    

    
    
    
    
    
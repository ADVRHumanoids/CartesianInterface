# -*- coding: utf-8 -*-
"""
Created on Fri Apr  6 11:05:36 2018

@author: arturo
"""

import rospy
import actionlib
import cartesian_interface.msg as cimsg
import cartesian_interface.srv as cisrv
import geometry_msgs.msg as geomsg
import std_srvs.srv as stdsrv
import XBotCore.msg as xbotmsg
import geometry_msgs.msg

class CogimonRosControl:
    
    def __init__(self):
        self.l_hand_client = actionlib.SimpleActionClient('/xbotcore/cartesian/LWrMot3/reach', cimsg.ReachPoseAction)
        self.r_hand_client = actionlib.SimpleActionClient('/xbotcore/cartesian/RWrMot3/reach', cimsg.ReachPoseAction)
        self.torso_client = actionlib.SimpleActionClient('/xbotcore/cartesian/DWYTorso/reach', cimsg.ReachPoseAction)
        
        self.clients = {}
        self.clients['l_hand'] = self.l_hand_client
        self.clients['r_hand'] = self.r_hand_client
        self.clients['torso'] = self.torso_client
        
        self.link_names = {}
        self.link_names['l_hand'] = 'LWrMot3'
        self.link_names['r_hand'] = 'RWrMot3'
        self.link_names['torso'] = 'DWYTorso'

        rospy.Subscriber('/xbotcore/cogimon/ft/l_arm_ft', geometry_msgs.msg.WrenchStamped, self.ft_callback, 'left_ft')        
        rospy.Subscriber('/xbotcore/cogimon/ft/r_arm_ft', geometry_msgs.msg.WrenchStamped, self.ft_callback, 'right_ft')     
        
        self.ft_values = {'left_ft': geometry_msgs.msg.Wrench(), 'right_ft': geometry_msgs.msg.Wrench()}
        
    def ft_callback(self, message, ft_name):
        self.ft_values[ft_name] = message.wrench
        
    def get_arm_ft_lr(self):
        return self.ft_values['left_ft'], self.ft_values['right_ft']
        
    def go_to(self, ee_name, pose, time, incremental = False):
        action_client = self.clients[ee_name]
        action_client.wait_for_server()
        goal = cimsg.ReachPoseGoal()
        goal.frames.append(pose)
        goal.time.append(time)
        goal.incremental = incremental
        action_client.send_goal(goal)
        
    def get_state(self, ee_name):
        action_client = self.clients[ee_name]
        if action_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        elif action_client.get_state() == actionlib.GoalStatus.ACTIVE:
            return 'active'
        elif action_client.get_state() == actionlib.GoalStatus.ABORTED:
            return 'aborted'
        elif action_client.get_state() == actionlib.GoalStatus.PREEMPTED:
            return 'preempted'
    
    def wait_for_result(self, ee_name):
        self.clients[ee_name].wait_for_result()
        
    def set_base_link(self, ee_name, new_base_link):
        
        rospy.wait_for_service('/xbotcore/cartesian/' + self.link_names[ee_name] +'/set_task_properties')
        try:
            set_task_info_srv = rospy.ServiceProxy('/xbotcore/cartesian/' + self.link_names[ee_name] +'/set_task_properties', cisrv.SetTaskInfo)
            req = cisrv.SetTaskInfoRequest()
            req.base_link = new_base_link
            res = cisrv.SetTaskInfoResponse()
            res = set_task_info_srv(req)
            print res

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
    def set_task_active(self, ee_name, active_flag):
        
        rospy.wait_for_service('/xbotcore/cartesian/' + self.link_names[ee_name] +'/activate_task')
        try:
            set_task_info_srv = rospy.ServiceProxy('/xbotcore/cartesian/' + self.link_names[ee_name] +'/activate_task', stdsrv.SetBool)
            req = stdsrv.SetBoolRequest()
            req = active_flag
            res = stdsrv.SetBoolResponse()
            res = set_task_info_srv(req)
            print res

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            
            
        
    def lhand_go_to(self, pose, time, incremental = False):
        self.go_to('l_hand', pose, time, incremental)

    def rhand_go_to(self, pose, time, incremental = False):
        self.go_to('r_hand', pose, time, incremental)

    def torso_go_to(self, pose, time, incremental = False):
        self.go_to('torso', pose, time, incremental)

    def lhand_getstate(self):
        return self.get_state('l_hand')
            
    def rhand_getstate(self):
        return self.get_state('r_hand')
        
    def torso_getstate(self):
        return self.get_state('torso')
            
    def lhand_wait_for_result(self):
        self.wait_for_result('l_hand')
       
    def rhand_wait_for_result(self):
        self.wait_for_result('r_hand')
        
    def torso_wait_for_result(self):
        self.wait_for_result('torso')
           
        
        
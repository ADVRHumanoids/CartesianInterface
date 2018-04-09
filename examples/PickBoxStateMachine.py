#!/usr/bin/env python


import rospy
import smach
import smach_ros
import CogimonRosControl 
import PickBoxPoses as poses
import geometry_msgs.msg as geomsg

robot = CogimonRosControl.CogimonRosControl()



# Initial state
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_down','accept_box','invalid_input'])

    def execute(self, userdata):
        rospy.loginfo('Entering IDLE mode')
        user_command = raw_input("Type next state: ") 
        if user_command == self.get_registered_outcomes()[0]:
            return self.get_registered_outcomes()[0]
        elif user_command == self.get_registered_outcomes()[1]:
            return self.get_registered_outcomes()[1]
        else:
            print "Invalid input"
            return 'invalid_input'



class Down(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], output_keys=['box_pos_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state DOWN')
        
        box_position = raw_input("Where is the box? [U/D]")
        box_z_offset = 0.3
        
        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (l_pose, r_pose) = poses.get_down_open_lr()

        if box_position == 'U':
            l_pose.position.z += box_z_offset
            r_pose.position.z += box_z_offset
            userdata.box_pos_output = box_z_offset
        
        robot.lhand_go_to(l_pose, 6.0)
        robot.rhand_go_to(r_pose, 6.0)
        robot.lhand_wait_for_result()
        robot.rhand_wait_for_result()
        
        return 'success'
        
        
class Closing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['failure', 'success'], input_keys=['box_pos_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing state CLOSING')
        
        # CALL ACTION TO CLOSE HANDS
        (l_pose, r_pose) = poses.get_down_close_lr()
        
        l_pose.position.z += userdata.box_pos_input
        r_pose.position.z += userdata.box_pos_input

        robot.lhand_go_to(l_pose, 2.0)
        robot.rhand_go_to(r_pose, 2.0)
        
        # MONITOR FT INSIDE A LOOP
        loop_rate = rospy.Rate(50)
        exit_loop = False
        closing_complete = False
        ft_threshold_reached = False
        
        while not exit_loop:
            closing_complete = robot.rhand_getstate() == 'succeeded' and robot.lhand_getstate() == 'succeeded'
            l_ft, r_ft = robot.get_arm_ft_lr()
            rospy.loginfo('FT Z value: left = %f, right = %f', l_ft.force.z, r_ft.force.z)
            ft_threshold_reached = abs(l_ft.force.z) > 20 and abs(r_ft.force.z) > 20
            exit_loop = closing_complete or ft_threshold_reached
            loop_rate.sleep()
        
        if ft_threshold_reached:
            return 'success'
        else:
            return 'failure'


class Up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'success_no_box'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UP')
        
        # CALL ACTION TO GO UP
        (l_pose, r_pose) = poses.get_up_close_lr()
        robot.lhand_go_to(l_pose, 6.0)
        robot.rhand_go_to(r_pose, 6.0)
        robot.lhand_wait_for_result()
        robot.rhand_wait_for_result()
        
        grasp_success = raw_input("Did we pick the box? [Y/n] ") == 'Y'
        
        if grasp_success:
            return 'success'
        else:
            return 'success_no_box'


class PassBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state PASS_BOX')
        
        robot.set_base_link('l_hand', 'DWYTorso')
        robot.set_base_link('r_hand', 'DWYTorso')
        
        pose = poses.get_pass_box_torso()
        
        robot.torso_go_to(pose, 3.0)
        robot.torso_wait_for_result()
        
        raw_input("Press ENTER to release box")
        
        # Release box
        delta_pose = geomsg.Pose()
        delta_pose.position.x = 0.0
        delta_pose.position.y = 0.05
        delta_pose.position.z = 0.0
        delta_pose.orientation.x = 0.0
        delta_pose.orientation.y = 0.0
        delta_pose.orientation.z = 0.0
        delta_pose.orientation.w = 1.0
        robot.lhand_go_to(delta_pose, 2.0, incremental=True)
        
        delta_pose = geomsg.Pose()
        delta_pose.position.x = 0.0
        delta_pose.position.y = -0.05
        delta_pose.position.z = 0.0
        delta_pose.orientation.x = 0.0
        delta_pose.orientation.y = 0.0
        delta_pose.orientation.z = 0.0
        delta_pose.orientation.w = 1.0
        robot.rhand_go_to(delta_pose, 2.0, incremental=True)
        
        robot.rhand_wait_for_result()
        robot.lhand_wait_for_result()
        
        
        return self.get_registered_outcomes()[0]
        
        
class BackHome(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state BACK_HOME')
        
        pose = geomsg.Pose()
        robot.torso_go_to(pose, 2.0)
        robot.torso_wait_for_result()
        
        rospy.sleep(rospy.Duration(1.0))
        
        robot.set_base_link('l_hand', 'world')
        robot.set_base_link('r_hand', 'world')
        
        return self.get_registered_outcomes()[0]
        


def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'go_down':'DOWN_SM', 
                                            'accept_box':'outcome5', 
                                            'invalid_input':'IDLE'})

        # Create the sub SMACH state machine
        sm_down = smach.StateMachine(outcomes=['closed', 'aborted'])
        sm_down.userdata.box_z_offset = 0.0

        # Open the container
        with sm_down:

            smach.StateMachine.add('DOWN', Down(), 
                                   transitions={'success':'CLOSING'}, 
                                   remapping={'box_pos_output':'box_z_offset'})
                                   
            smach.StateMachine.add('CLOSING', Closing(), 
                                   transitions={'success':'closed', 
                                                'failure':'aborted'}, 
                                   remapping={'box_pos_input':'box_z_offset'})
                                   
                                   

        smach.StateMachine.add('DOWN_SM', sm_down,
                               transitions={'closed':'UP', 
                                            'aborted':'UP'})
                                            
        smach.StateMachine.add('UP', Up(),
                               transitions={'success':'PASS_BOX', 
                                            'success_no_box':'IDLE'})
                               
        smach.StateMachine.add('PASS_BOX', PassBox(),
                               transitions={'success':'BACK_HOME'})
                               
        smach.StateMachine.add('BACK_HOME', BackHome(),
                               transitions={'success':'IDLE'})
                                            
                                            

    
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
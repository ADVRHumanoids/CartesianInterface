#!/usr/bin/env python


import rospy
import smach
import smach_ros
from cartesian_interface import pyci
from cartesian_interface import roscpp_utils as roscpp
from cartesian_interface.affine3 import Affine3
import py_demo_example_poses as poses
import sys
import threading
import numpy
import time

robot = None
fl_wheel = 'wheel_1'
fr_wheel = 'wheel_2'
rr_wheel = 'wheel_4'
rl_wheel = 'wheel_3'
fl_ankle = 'ankle2_1'
fr_ankle = 'ankle2_2'
rr_ankle = 'ankle2_4'
rl_ankle = 'ankle2_3'
waist = 'pelvis'
fl_sgn = numpy.array([ 1,  1, 1])
fr_sgn = numpy.array([ 1, -1, 1])
rr_sgn = numpy.array([-1, -1, 1])
rl_sgn = numpy.array([-1,  1, 1])

class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Idle.. ')
        raw_input('Press ENTER to continue..')

        return 'done'

class GetStable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GET STABLE')


        robot.loadController('WheeledMotion')

        # CALL ACTION TO GO DOWN AND WAIT FOR RESULT
        (fl, fr, rr, rl, wa) = poses.get_pose_0123wa_stable()

        robot.setTargetPose(fl_wheel, fl, 5.0)
        robot.setTargetPose(fr_wheel, fr, 5.0)
        robot.setTargetPose(rr_wheel, rr, 5.0)
        robot.setTargetPose(rl_wheel, rl, 5.0)
        robot.setTargetPose(waist, wa, 2.0)

        robot.waitReachCompleted(waist)
        robot.waitReachCompleted(fl_wheel)
        robot.waitReachCompleted(fr_wheel)
        robot.waitReachCompleted(rr_wheel)
        robot.waitReachCompleted(rl_wheel)
        
        raw_input('Press ENTER to continue..')

        return 'done'


class VelocityPublisher:
    def __init__(self, tname):
        self.vref = numpy.zeros(6)
        self.lock = threading.Lock()
        self.task_name = tname     
        self.timer = None
        
    def run(self, event):
        self.lock.acquire()
        robot.setPoseReference(self.task_name, Affine3(), self.vref)
        self.lock.release()
        
    def set_vref(self, vref):
        with self.lock:
            self.vref = vref.copy()
    
    def start(self):
        self.running = True
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.run)
            
    def stop(self):
        self.timer.shutdown()
        rospy.loginfo('VelocityPublisher returned')

class StartExecution(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state START')

        robot.setControlMode(waist, pyci.ControlType.Velocity)
        vel_pub = VelocityPublisher(waist) # keep publishing vref from a separate thread
        vref = numpy.array([0, 0, 0, 0, 0, 1.0])
        vel_pub.set_vref(vref)
        vel_pub.start()
        
        raw_input('Press ENTER to start lifting..')
        
        # Raise FR
        (fl, fr, rr, rl, _) = poses.get_pose_0123wa_stable()
        rr = Affine3([-0.13, -0.37, -0.60])
        fl = Affine3([ 0.52,  0.22, -0.60])
        
        robot.setTargetPose(fl_wheel, fl, 5.0)
        robot.setTargetPose(fr_wheel, fr, 5.0)
        robot.setTargetPose(rr_wheel, rr, 5.0)
        robot.setTargetPose(rl_wheel, rl, 5.0)

        robot.waitReachCompleted(fl_wheel)
        robot.waitReachCompleted(fr_wheel)
        robot.waitReachCompleted(rr_wheel)
        robot.waitReachCompleted(rl_wheel)

        wp1 = pyci.WayPoint(Affine3([0, 0, 0.1]), 2.0)
        wp2 = pyci.WayPoint(Affine3([0, 0, 0.0]), 4.0)
        robot.setWaypoints('ankle2_2', [wp1, wp2], True)        
        robot.waitReachCompleted('ankle2_2')
        
        
        raw_input('Press ENTER to stop spinning..')
        
        vel_pub.stop()

        return 'done'



def main():
    
    node_name = 'lift_leg_state_machine'

    roscpp.init(node_name, sys.argv)    
    rospy.init_node(node_name, sys.argv)
    
    
    global robot
    robot = pyci.CartesianInterfaceRos()

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('GET_STABLE', GetStable(),
                               transitions={'done': 'EXEC'})

        smach.StateMachine.add('EXEC', StartExecution(),
                               transitions={'done': 'GET_STABLE'})



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
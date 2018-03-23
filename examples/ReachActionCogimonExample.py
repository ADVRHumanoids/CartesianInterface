import rospy
import actionlib
import cartesian_interface.msg as cimsg
import geometry_msgs.msg as geomsg
import std_srvs.srv as stdsrv

def wholebody_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client_1 = actionlib.SimpleActionClient('/xbotcore/cartesian/LWrMot3/reach', cimsg.ReachPoseAction)

    # Call service to disable control of right arm
    rospy.wait_for_service('/xbotcore/cartesian/RWrMot3/activate_task')
    try:
        activate_task = rospy.ServiceProxy('/xbotcore/cartesian/RWrMot3/activate_task', stdsrv.SetBool)
        resp1 = activate_task(False)
        print resp1.message

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


    # Waits until the action server has started up and started
    # listening for goals.
    client_1.wait_for_server()


    # Creates a goal to send to the action server.
    goal_1 = cimsg.ReachPoseGoal()
    pose = geomsg.Pose()
    pose.position.x = 0.30
    pose.position.y = 0.320
    pose.position.z = -0.76
    pose.orientation.x = 0.08
    pose.orientation.y = -0.13
    pose.orientation.z = 0.1
    pose.orientation.w = 0.98

    goal_1.frames.append(pose)
    goal_1.time.append(10.0)
    goal_1.incremental = False # Move relative to current pose


    # Sends the goal to the action server.
    client_1.send_goal(goal_1)

    # Waits for the server to finish performing the action.
    client_1.wait_for_result()


if __name__ == '__main__':

       try:
           # Initializes a rospy node so that the SimpleActionClient can
           # publish and subscribe over ROS.
           rospy.init_node('reach_pose_action_client')
           result = wholebody_client()

       except rospy.ROSInterruptException:
           print("program interrupted before completion")

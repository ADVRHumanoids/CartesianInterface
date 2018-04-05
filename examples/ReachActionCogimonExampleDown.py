import rospy
import actionlib
import cartesian_interface.msg as cimsg
import geometry_msgs.msg as geomsg
import std_srvs.srv as stdsrv

def wholebody_client():
    # Creates the SimpleActionClient, passing the type of the action
    # to the constructor.
    client_1 = actionlib.SimpleActionClient('/xbotcore/cartesian/LWrMot3/reach', cimsg.ReachPoseAction)
    client_2 = actionlib.SimpleActionClient('/xbotcore/cartesian/RWrMot3/reach', cimsg.ReachPoseAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client_1.wait_for_server()
    client_2.wait_for_server()


    # Creates a goal to send to the action server.
    goal_1 = cimsg.ReachPoseGoal()
    pose = geomsg.Pose()
    pose.position.x = 0.20
    pose.position.y = 0.35
    pose.position.z = -0.6
    pose.orientation.x = 0.0
    pose.orientation.y = -0.4
    pose.orientation.z = 0.0
    pose.orientation.w = 0.9

    goal_1.frames.append(pose)
    pose_2 = geomsg.Pose()
    pose_2.position.x = 0.20
    pose_2.position.y = 0.24
    pose_2.position.z = -0.6
    pose_2.orientation.x = 0.0
    pose_2.orientation.y = -0.4
    pose_2.orientation.z = 0.0
    pose_2.orientation.w = 0.9
    goal_1.frames.append(pose_2)
    
    
    
    goal_1.time.append(12.0)
    goal_1.time.append(15.0)
    goal_1.incremental = False # Move relative to current pose


    # Sends the goal to the action server.
    client_1.send_goal(goal_1)


    # Sends the goal to the action server.
    goal_2 = cimsg.ReachPoseGoal()
    pose_2 = geomsg.Pose()
    pose_2.position.x = 0.20
    pose_2.position.y = -0.35
    pose_2.position.z = -0.6
    pose_2.orientation.x = 0.0
    pose_2.orientation.y = -0.4
    pose_2.orientation.z = 0.0
    pose_2.orientation.w = 0.9

    goal_2.frames.append(pose_2)
    
    pose_2 = geomsg.Pose()
    pose_2.position.x = 0.20
    pose_2.position.y = -0.24
    pose_2.position.z = -0.6
    pose_2.orientation.x = 0.0
    pose_2.orientation.y = -0.4
    pose_2.orientation.z = 0.0
    pose_2.orientation.w = 0.9
    
    goal_2.frames.append(pose_2)
    
    goal_2.time.append(12.0)
    goal_2.time.append(15.0)
    goal_2.incremental = False  # Move relative to current pose

    client_2.send_goal(goal_2)

    # Waits for the server to finish performing the action.
    client_1.wait_for_result()
    client_2.wait_for_result()


if __name__ == '__main__':

       try:
           # Initializes a rospy node so that the SimpleActionClient can
           # publish and subscribe over ROS.
           rospy.init_node('reach_pose_action_client')
           result = wholebody_client()

       except rospy.ROSInterruptException:
           print("program interrupted before completion")

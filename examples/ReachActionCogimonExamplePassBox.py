import rospy
import actionlib
import cartesian_interface.msg as cimsg
import cartesian_interface.srv as cisrv
import geometry_msgs.msg as geomsg
import std_srvs.srv as stdsrv


def wholebody_client():
    
    # Change base link to DWYTorso
    rospy.wait_for_service('/xbotcore/cartesian/RWrMot3/set_task_properties')
    try:
        set_task_info_srv = rospy.ServiceProxy('/xbotcore/cartesian/RWrMot3/set_task_properties', cisrv.SetTaskInfo)
        req = cisrv.SetTaskInfoRequest()
        req.base_link = 'DWYTorso'
        res = cisrv.SetTaskInfoResponse()
        res = set_task_info_srv(req)
        print res

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        
    # Change base link to DWYTorso
    rospy.wait_for_service('/xbotcore/cartesian/LWrMot3/set_task_properties')
    try:
        set_task_info_srv = rospy.ServiceProxy('/xbotcore/cartesian/LWrMot3/set_task_properties', cisrv.SetTaskInfo)
        req = cisrv.SetTaskInfoRequest()
        req.base_link = 'DWYTorso'
        res = cisrv.SetTaskInfoResponse()
        res = set_task_info_srv(req)
        print res

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        
    # Reach action client
    action_client = actionlib.SimpleActionClient('/xbotcore/cartesian/DWYTorso/reach', cimsg.ReachPoseAction)
    action_client.wait_for_server()
    
    # Define goal
    goal = cimsg.ReachPoseGoal()
    pose = geomsg.Pose()
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.7
    pose.orientation.w = 1.0
    goal.frames.append(pose)
    goal.time.append(3.0)
    goal.incremental = True
    
    action_client.send_goal(goal)
    action_client.wait_for_result()
    
    
    


if __name__ == '__main__':

       try:
           # Initializes a rospy node so that the SimpleActionClient can
           # publish and subscribe over ROS.
           rospy.init_node('reach_pose_action_client')
           result = wholebody_client()

       except rospy.ROSInterruptException:
           print("program interrupted before completion")
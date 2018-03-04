#include <CartesianInterface/ros/RosServerClass.h>

using namespace XBot::Cartesian;

void RosServerClass::__generate_state_broadcasting()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "/xbotcore/cartesian/" + ee_name + "/state";
        
        ros::Publisher pub = _nh.advertise<geometry_msgs::PoseStamped>(topic_name, 
                                                                        1);
        
        _state_pub.push_back(pub);
    }
}



void RosServerClass::online_position_reference_cb(const geometry_msgs::PoseStampedConstPtr& msg, 
                                                  const std::string& ee_name)
{
    
    Eigen::Affine3d T;
    tf::poseMsgToEigen(msg->pose, T);
    
    _cartesian_interface->setPoseReference(ee_name, T);
}


void RosServerClass::__generate_online_pos_topics()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string topic_name = "/xbotcore/cartesian/" + ee_name + "/reference";
        
        auto cb = std::bind(&RosServerClass::online_position_reference_cb, this, std::placeholders::_1, ee_name);
        
        ros::Subscriber sub = _nh.subscribe<geometry_msgs::PoseStamped>(topic_name, 
                                                                        1, cb);
        
        _pos_sub.push_back(sub);
    }
}

void RosServerClass::__generate_reach_pose_action_servers()
{
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        std::string action_name = "/xbotcore/cartesian/" + ee_name + "/reach";

        _action_servers.emplace_back( new ActionServer(_nh, action_name, false) );
        
        _action_servers.back()->start();
        
        _is_action_active.push_back(false);
    }
}

void RosServerClass::manage_reach_actions()
{
    /* Poll */
    for(int i = 0; i < _action_servers.size(); i++)
    {
        ActionServerPtr as = _action_servers[i];
        
        const std::string& ee_name = _cartesian_interface->getTaskList()[i];
        
        CartesianInterface::State current_state = _cartesian_interface->getTaskState(ee_name);
        
        if(as->isNewGoalAvailable() && current_state != CartesianInterface::State::Reaching)
        {
            if(!_is_action_active[i])
            {
                auto goal = as->acceptNewGoal();
                
                Eigen::Affine3d T_ref;
                tf::poseMsgToEigen(goal->frames.back(), T_ref);
                
                Eigen::Affine3d base_T_ee;
                 _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
                 
                if(goal->incremental)
                {
                    T_ref = base_T_ee *  T_ref;
                }
                
                if(!_cartesian_interface->setTargetPose(ee_name, T_ref, goal->time.back()))
                {
                    as->setAborted(cartesian_interface::ReachPoseResult(), "Internal error");
                }
                
            }
        }
        
        if(as->isPreemptRequested())
        {
            _cartesian_interface->abort(ee_name);
            
            cartesian_interface::ReachPoseResult result;
            Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
            _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
            _cartesian_interface->getPoseReference(ee_name, base_T_ref);
            ref_T_ee = base_T_ref.inverse() * base_T_ee;
            
            tf::poseEigenToMsg(ref_T_ee, result.final_error);
            result.position_error_norm = ref_T_ee.translation().norm();
            result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();
            
            as->setPreempted(result);
        }
        
        if(as->isActive())
        {
            if(current_state == CartesianInterface::State::Online)
            {
                cartesian_interface::ReachPoseResult result;
                Eigen::Affine3d base_T_ee, base_T_ref, ref_T_ee;
                _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
                _cartesian_interface->getPoseReference(ee_name, base_T_ref);
                ref_T_ee = base_T_ref.inverse() * base_T_ee;
                
                tf::poseEigenToMsg(ref_T_ee, result.final_error);
                result.position_error_norm = ref_T_ee.translation().norm();
                result.orientation_error_angle = Eigen::AngleAxisd(ref_T_ee.linear()).angle();
                
                as->setSucceeded(result);
            }
        }
    }
}

void RosServerClass::publish_state()
{
    int i = 0;
    for(std::string ee_name : _cartesian_interface->getTaskList())
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        Eigen::Affine3d base_T_ee;
        _cartesian_interface->getCurrentPose(ee_name, base_T_ee);
        tf::poseEigenToMsg(base_T_ee, msg.pose);
        _state_pub[i].publish(msg);
        i++;
    }
}

void RosServerClass::run()
{
    ros::spinOnce();
    
    manage_reach_actions();
    
    publish_state();
}

RosServerClass::RosServerClass(CartesianInterface::Ptr intfc):
    _cartesian_interface(intfc)
{
    __generate_online_pos_topics();
    __generate_reach_pose_action_servers();
    __generate_state_broadcasting();
}
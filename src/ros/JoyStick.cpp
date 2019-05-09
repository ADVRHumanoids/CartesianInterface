#include <cartesian_interface/joystick/JoyStick.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <cartesian_interface/CartesioJoyStick.h>
#include <algorithm>


#define MAX_SPEED_SF 1.0
#define MIN_SPEED_SF 0.0

using namespace XBot::Cartesian;

JoyStick::JoyStick(const std::vector<std::string>& distal_links, 
                   const std::vector<std::string>& base_links, 
                   std::string tf_prefix):
    _tf_prefix(tf_prefix),
    _distal_links(distal_links),
    _base_links(base_links),
    _robot_base_link("base_link"),
    _selected_task(0),
    _linear_speed_sf(0.2),
    _angular_speed_sf(0.2),
    _local_ctrl(-1), _base_ctrl(-1),
    _nh("cartesian")
{
    _twist.setZero();
    _twist_mask.setOnes();
    _twist_filt.setDamping(1.0);
    _twist_filt.setOmega(2.0 * M_PI * 2.0);
    _twist_filt.setTimeStep(1./30.);
    
    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStick::joyCallback, this);

    _joy_audio_pub = _nh.advertise<std_msgs::String>("audio", 1);

    ros::ServiceClient srv_client;
    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        srv_client = _nh.serviceClient<cartesian_interface::SetTaskInfo>(_distal_links[i] + "/set_task_properties");
        _set_properties_service_clients.push_back(srv_client);
    }

    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        srv_client = _nh.serviceClient<cartesian_interface::GetTaskInfo>(_distal_links[i] + "/get_task_properties");
        _get_properties_service_clients.push_back(srv_client);
    }

    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        srv_client = _nh.serviceClient<std_srvs::SetBool>(_distal_links[i] + "/activate_task");
        _task_active_service_client.push_back(srv_client);
    }

    ros::Publisher ref_pose_pub;
    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        std::string topic_name = _distal_links[i] + "/velocity_reference";
        ref_pose_pub = _nh.advertise<geometry_msgs::TwistStamped>(topic_name, 1);
        _ref_pose_pubs.push_back(ref_pose_pub);
    }


    _joystick_status_pub = _nh.advertise<cartesian_interface::CartesioJoyStick>("joystick_status", 1);

    _joystick_set_active_task_service = _nh.advertiseService("SetJoystickActiveTask",
                                                             &JoyStick::setActiveTask, this);

    std::stringstream ss;
    ss<<"Selected Task \n               distal_link: "<<_distal_links[_selected_task].c_str()<<std::endl;
    ss<<"               base_link:   "<<_base_links[_selected_task].c_str()<<std::endl;
    if(_local_ctrl == 1)
        ss<<"               LOCAL ctrl"<<std::endl;
    else
        ss<<"               GLOBAL ctrl"<<std::endl;
    if(_base_ctrl == 1)
    {
        ss<<"               BASE ctrl ON"<<std::endl;
        ss<<"               robot base_link is: "<<_robot_base_link.c_str()<<std::endl;
    }
    else
        ss<<"               BASE ctrl OFF"<<std::endl;
    ROS_INFO("%s", ss.str().c_str());
}



void XBot::Cartesian::JoyStick::setTwistMask(const Eigen::Vector6i& mask)
{
    _twist_mask = mask;
}


JoyStick::~JoyStick()
{

}

void JoyStick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    /* Print documentation to terminal */
    if(joy->buttons[6])
    {
        ROS_INFO(Doc().c_str());
    }
    
    /* Change selected task */
    if(joy->buttons[4])
    {
        _selected_task--;
    }

    if(joy->buttons[5])
    {
        _selected_task++;
    }
    
    if(_selected_task < 0)
    {
        _selected_task = _distal_links.size()-1;
    }
    
    _selected_task = _selected_task % _distal_links.size();

    /* Print info on selected task, if changed */
    if(joy->buttons[4] || joy->buttons[5])
    {
        std::stringstream ss;
        ss<<"Selected Task \n               distal_link: "<<_distal_links[_selected_task].c_str()<<std::endl;
        ss<<"               base_link:   "<<_base_links[_selected_task].c_str()<<std::endl;
        ROS_INFO("%s", ss.str().c_str());
        
        std_msgs::String msg;
        msg.data = _distal_links[_selected_task];
        _joy_audio_pub.publish(msg);

    }

    /* Activate control */
    if(joy->buttons[7])
    {
        setVelocityCtrl();
    }
    
    /* Change speed scale (linear) */
    if(joy->axes[2] <= -0.99)
    {
        if(joy->buttons[2])
        {
            _linear_speed_sf -= 0.05;
        }
        
        if(joy->buttons[3])
        {
            _linear_speed_sf += 0.05;
        }

        /* Clamp resulting value */
        _linear_speed_sf = std::max(MIN_SPEED_SF, std::min(_linear_speed_sf, MAX_SPEED_SF));
    }

    /* Print info on speed scale change */
    if(joy->axes[2] <= -0.99 && (joy->buttons[2] || joy->buttons[3]))
    {
        ROS_INFO("_linear_speed_sf: %f", _linear_speed_sf);
    }

    /* Change speed scale (angular) */
    if(joy->axes[5] <= -0.99)
    {
        if(joy->buttons[2])
        {
            _angular_speed_sf -= 0.05;
        }
        
        if(joy->buttons[3])
        {
            _angular_speed_sf += 0.05;
        }

        /* Clamp resulting value */
        _angular_speed_sf = std::max(MIN_SPEED_SF, std::min(_angular_speed_sf, MAX_SPEED_SF));
    }

    /* Print info on speed scale change */
    if(joy->axes[5] <= -0.99 && (joy->buttons[2] || joy->buttons[3]))
    {
        ROS_INFO("_angular_speed_sf: %f", _angular_speed_sf);
    }

    /* Define desired twist */
    _twist[0] = _linear_speed_sf * joy->axes[1];
    _twist[1] = _linear_speed_sf * joy->axes[0];
    _twist[2] = _linear_speed_sf * joy->axes[7];
    _twist[3] = _angular_speed_sf * joy->axes[6];
    _twist[4] = _angular_speed_sf * joy->axes[4];
    _twist[5] = _angular_speed_sf * joy->axes[3];
    
    /* Apply mask */
    _twist = _twist.cwiseProduct(_twist_mask.cast<double>());

    /* Toggle base control (control w.r.t. task's base link) */
    _desired_twist.header.frame_id = ""; 
    
    if(joy->buttons[3] && joy->axes[5] > -0.99 && joy->axes[2] > -0.99)
    {
        _base_ctrl *= -1;
        
        if(_base_ctrl == 1)
        {
            std::stringstream ss;
            ss<<"               BASE ctrl ON"<<std::endl;
            ss<<"               robot base_link is: "<<_robot_base_link.c_str()<<std::endl;
            ROS_INFO("%s", ss.str().c_str());
        }
        else
        {
            ROS_INFO("              \n    BASE ctrl OFF");
        }
        
    }

    if(_base_ctrl == 1)
    {
        twistInBase();
    }
    
    /* Toggle local control */
    if(joy->buttons[0])
    {
        _local_ctrl *= -1;
        
        if(_local_ctrl == 1)
        {
            ROS_INFO("              \n    LOCAL ctrl");
        }
        else
        {
            ROS_INFO("              \n    GLOBAL ctrl");
        }
    }
    
    if(_local_ctrl == 1)
    {
        localCtrl();
    }

/* REMOVED TOO DANGEROUS */
//    if(joy->buttons[1])
//    {
//        activateDeactivateTask();
//    }
}

void JoyStick::sendVelRefs()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);
    
    _twist_filt.process(_twist);
    
    _desired_twist.twist.linear.x  = _twist_filt.getOutput()[0];
    _desired_twist.twist.linear.y  = _twist_filt.getOutput()[1];
    _desired_twist.twist.linear.z  = _twist_filt.getOutput()[2];
    _desired_twist.twist.angular.x = _twist_filt.getOutput()[3];
    _desired_twist.twist.angular.y = _twist_filt.getOutput()[4];
    _desired_twist.twist.angular.z = _twist_filt.getOutput()[5];
    
    if(srv.response.control_mode == "Velocity")
    {
        _ref_pose_pubs[_selected_task].publish(_desired_twist);
    }
}

void JoyStick::setVelocityCtrl()
{
    cartesian_interface::SetTaskInfo srv;

    srv.request.control_mode = "Velocity";
    _set_properties_service_clients[_selected_task].call(srv);

}

void JoyStick::localCtrl()
{
    _desired_twist.header.frame_id =  _distal_links[_selected_task];
}

void XBot::Cartesian::JoyStick::twistInBase()
{
    _desired_twist.header.frame_id = "base_link";
}

bool XBot::Cartesian::JoyStick::setActiveTask(cartesian_interface::SetJoystickActiveTaskRequest &req,
                                              cartesian_interface::SetJoystickActiveTaskResponse &res)
{
    std::string task = req.active_task;
    if(!(std::find_if(_distal_links.begin(), _distal_links.end(),
                    [&](const std::string& arg) { return arg == task; }) != _distal_links.end()))
    {
        ROS_ERROR("Requested task %s is not available!", task.c_str());
        res.success = false;
        res.error_message = "Requested task" + task + "is not available!";
        return false;
    }

    std::vector<std::string>::iterator it = std::find(_distal_links.begin(), _distal_links.end(), task);
    _selected_task = std::distance(_distal_links.begin(), it);

    std::stringstream ss;
    ss<<"Selected Task \n               distal_link: "<<_distal_links[_selected_task].c_str()<<std::endl;
    ss<<"               base_link:   "<<_base_links[_selected_task].c_str()<<std::endl;
    ROS_INFO("%s", ss.str().c_str());

    std_msgs::String msg;
    msg.data = _distal_links[_selected_task];
    _joy_audio_pub.publish(msg);

    res.success = true;
    res.error_message = "";
    return true;
}


//void JoyStick::activateDeactivateTask()
//{
//    cartesian_interface::GetTaskInfo srv;
//    _get_properties_service_clients[_selected_task].call(srv);

//    std_srvs::SetBool srv2;

//    if(srv.response.control_mode.compare("Disabled") == 0){
//        srv2.request.data = true;
//        ROS_INFO("              \n    ENABLING task");}
//    else{
//        srv2.request.data = false;
//        ROS_INFO("              \n    DISABLING task");}

//    _task_active_service_client[_selected_task].call(srv2);
//}

std::string JoyStick::getRobotBaseLinkCtrlFrame()
{
    return _robot_base_link;
}

void JoyStick::setRobotBaseLinkCtrlFrame(const std::string& robot_base_link)
{
    _robot_base_link = robot_base_link;
}

bool XBot::Cartesian::JoyStick::updateStatus()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);

    std::string link = srv.response.base_link;
    if(link == "world")
        link = "world_odom";


    if(link != _base_links[_selected_task])
    {
        ROS_WARN("base_link of task %s changed outside joystick", _distal_links[_selected_task].c_str());
        ROS_WARN("from %s to %s\n", _base_links[_selected_task].c_str(), link.c_str());
        _base_links[_selected_task] = link;

    }

    return true;
}

void XBot::Cartesian::JoyStick::broadcastStatus()
{
    cartesian_interface::CartesioJoyStick msg;
    msg.max_angular_speed = _angular_speed_sf;
    msg.max_linear_speed = _linear_speed_sf;

    for(unsigned int i = 0; i < _twist_mask.size(); ++i)
        msg.twist_mask[i] = _twist_mask[i];

    _joystick_status_pub.publish(msg);
}


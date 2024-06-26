#include <cartesian_interface/joystick/JoyStick.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>
#include <cartesian_interface/JoystickStatus.h>
#include <algorithm>

#define MAX_SPEED_SF 1.0
#define MIN_SPEED_SF 0.0

using namespace XBot::Cartesian;

JoyStick::JoyStick(std::shared_ptr<RosClient> ci_ros,
                   std::string tf_prefix):
    _tf_prefix(tf_prefix),
    _ci(ci_ros),
    _robot_base_link("base_link"),
    _selected_task(0),
    _linear_speed_sf(0.2),
    _angular_speed_sf(0.2),
    _local_ctrl(-1), _base_ctrl(-1),
    _nh("cartesian")
{
    _ci->set_async_mode(false);

    // fill distal and base links for each cartesian task
    for(auto tname : _ci->getTaskList())
    {
        auto task = _ci->getTask(tname);

        if(auto cart = std::dynamic_pointer_cast<CartesianTask>(task))
        {
            auto distal = cart->getDistalLink();
            auto base = cart->getBaseLink();

            _distal_links.push_back(distal);
            _base_links.push_back(base);
        }
    }

    _twist.setZero();
    _twist_mask.setOnes();
    _twist_filt.setDamping(1.0);
    _twist_filt.setOmega(2.0 * M_PI * 2.0);
    _twist_filt.setTimeStep(1./30.);
    
    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStick::joyCallback, this);

    _joy_audio_pub = _nh.advertise<std_msgs::String>("audio", 1);

    _joystick_status_pub = _nh.advertise<cartesian_interface::JoystickStatus>("joystick/joystick_status", 1, true);
    _joystick_set_active_task_service = _nh.advertiseService("joystick/set_active_task",
                                                             &JoyStick::setActiveTask, this);

    _joystick_set_task_max_speed_service = _nh.advertiseService("joystick/set_max_speed",
                                                                &JoyStick::setMaxSpeed, this);
    _joystick_set_task_base_frame_service = _nh.advertiseService("joystick/set_base_frame",
                                                                &JoyStick::setBaseFrame, this);

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

    broadcastStatus();

    _ci->set_async_mode(true);
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

    broadcastStatus();
}

void JoyStick::sendVelRefs()
{
    _twist_filt.process(_twist);
    
    if(ControlType::Velocity == _ci->getControlMode(_distal_links.at(_selected_task)))
    {
        _ci->setVelocityReference(_distal_links.at(_selected_task),
                                  _twist_filt.getOutput(),
                                  _desired_twist.header.frame_id);
    }
}

void JoyStick::setVelocityCtrl()
{
    auto ctrl_mode = _ci->getControlMode(_distal_links.at(_selected_task));

    if(ctrl_mode == ControlType::Velocity)
    {
        _ci->setControlMode(_distal_links.at(_selected_task), ControlType::Position);
        ROS_INFO("Set 'Position' control mode");
    }
    else if(ctrl_mode == ControlType::Position)
    {
        _ci->setControlMode(_distal_links.at(_selected_task), ControlType::Velocity);
        ROS_INFO("Set 'Velocity' control mode");
    }
}

void JoyStick::localCtrl()
{
    _desired_twist.header.frame_id =  _distal_links[_selected_task];
}

void XBot::Cartesian::JoyStick::twistInBase()
{
    _base_ctrl = 1;
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

    broadcastStatus();

    res.success = true;
    res.error_message = "";
    return true;
}

bool XBot::Cartesian::JoyStick::setMaxSpeed(cartesian_interface::SetJoystickTaskMaxSpeedRequest &req,
                                            cartesian_interface::SetJoystickTaskMaxSpeedResponse &res)
{
    _linear_speed_sf = req.max_linear_speed;
    _angular_speed_sf = req.max_angular_speed;

    /* Clamp resulting value */
    _linear_speed_sf  = std::max(MIN_SPEED_SF, std::min(_linear_speed_sf, MAX_SPEED_SF));
    _angular_speed_sf = std::max(MIN_SPEED_SF, std::min(_angular_speed_sf, MAX_SPEED_SF));

    ROS_INFO("_linear_speed_sf: %f", _linear_speed_sf);
    ROS_INFO("_angular_speed_sf: %f", _angular_speed_sf);

    broadcastStatus();

    return true;
}

bool XBot::Cartesian::JoyStick::setBaseFrame(cartesian_interface::SetJoystickTaskBaseFrameRequest &req,
                                             cartesian_interface::SetJoystickTaskBaseFrameResponse &res)
{
    ROS_INFO("Requested base frame: '%s'", req.base_frame.c_str());
    
    res.error_message = "";
    res.success = false;
    if(req.base_frame == "global")
    {
        _base_ctrl = -1;
        _local_ctrl = -1;
        _desired_twist.header.frame_id = "";
        res.success = true;
    }
    else if(req.base_frame == "local")
    {
        _base_ctrl = -1;
        _local_ctrl = 1;
        _desired_twist.header.frame_id =  _distal_links[_selected_task];
        res.success = true;
    }
    else if(req.base_frame == "base_link")
    {
        twistInBase();
        res.success = true;
    }

    broadcastStatus();

    if(!res.success)
    {
        res.error_message = req.base_frame + " not allowed. Please choose among 'global', 'local', 'base_link'";
    }

    return true;
}


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
    _ci->update(0, 0);

    std::string link = _ci->getBaseLink(_distal_links[_selected_task]);

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
    cartesian_interface::JoystickStatus msg;
    msg.max_angular_speed = _angular_speed_sf;
    msg.max_linear_speed = _linear_speed_sf;
    msg.active_task = _distal_links[_selected_task];

    for(int i = 0; i < _twist_mask.size(); ++i)
    {
        msg.twist_mask[i] = _twist_mask[i];
    }

    _joystick_status_pub.publish(msg);
}


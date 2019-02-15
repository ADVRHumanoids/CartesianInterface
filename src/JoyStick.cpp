#include <cartesian_interface/joystick/JoyStick.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>


#define MAX_SPEED_SF 1.0
#define MIN_SPEED_SF 0.0

using namespace XBot::Cartesian;

JoyStick::JoyStick(const std::vector<std::string> &distal_links, const std::vector<std::string> &base_links, std::string tf_prefix):
    _tf_prefix(tf_prefix),
    _distal_links(distal_links),
    _base_links(base_links),
    _robot_base_link("base_link"),
    _selected_task(0),
    _linear_speed_sf(0.2),
    _angular_speed_sf(0.2),
    _twist(6), _local_ctrl(-1), _base_ctrl(-1),
    _nh("cartesian")
{
    _twist.setZero(6);
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

JoyStick::~JoyStick()
{

}

void JoyStick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[6])
        ROS_INFO(Doc().c_str());

    if(joy->buttons[4])
        _selected_task--;

    if(joy->buttons[5])
        _selected_task++;

    if(_selected_task < 0)
        _selected_task = _distal_links.size()-1;

    _selected_task = _selected_task%_distal_links.size();

    if(joy->buttons[4] || joy->buttons[5]){
        std::stringstream ss;
        ss<<"Selected Task \n               distal_link: "<<_distal_links[_selected_task].c_str()<<std::endl;
        ss<<"               base_link:   "<<_base_links[_selected_task].c_str()<<std::endl;
        ROS_INFO("%s", ss.str().c_str());
        
        std_msgs::String msg;
        msg.data = _distal_links[_selected_task];
        _joy_audio_pub.publish(msg);

    }

    if(joy->buttons[7])
        setVelocityCtrl();

    if(joy->axes[2] <= -0.99)
    {
        if(joy->buttons[2])
            _linear_speed_sf -= 0.05;
        if(joy->buttons[3])
            _linear_speed_sf += 0.05;

        if(_linear_speed_sf < MIN_SPEED_SF)
            _linear_speed_sf = MIN_SPEED_SF;
        if(_linear_speed_sf > MAX_SPEED_SF)
            _linear_speed_sf = MAX_SPEED_SF;
    }

    if(joy->axes[2] <= -0.99 && (joy->buttons[2] || joy->buttons[3]))
        ROS_INFO("_linear_speed_sf: %f", _linear_speed_sf);

    if(joy->axes[5] <= -0.99)
    {
        if(joy->buttons[2])
            _angular_speed_sf -= 0.05;
        if(joy->buttons[3])
            _angular_speed_sf += 0.05;

        if(_angular_speed_sf < MIN_SPEED_SF)
            _angular_speed_sf = MIN_SPEED_SF;
        if(_angular_speed_sf > MAX_SPEED_SF)
            _angular_speed_sf = MAX_SPEED_SF;
    }

    if(joy->axes[5] <= -0.99 && (joy->buttons[2] || joy->buttons[3]))
        ROS_INFO("_angular_speed_sf: %f", _angular_speed_sf);


    _twist.setZero(6);
    _twist[0] = _linear_speed_sf * joy->axes[1];
    _twist[1] = _linear_speed_sf * joy->axes[0];
    _twist[2] = _linear_speed_sf * joy->axes[7];
    _twist[3] = _angular_speed_sf * joy->axes[6];
    _twist[4] = _angular_speed_sf * joy->axes[4];
    _twist[5] = _angular_speed_sf * joy->axes[3];

    if(joy->buttons[3] && joy->axes[5] > -0.99 && joy->axes[2] > -0.99){
        _base_ctrl *= -1;
        if(_base_ctrl == 1)
        {
            std::stringstream ss;
            ss<<"               BASE ctrl ON"<<std::endl;
            ss<<"               robot base_link is: "<<_robot_base_link.c_str()<<std::endl;
            ROS_INFO("%s", ss.str().c_str());
        }
        else
            ROS_INFO("              \n    BASE ctrl OFF");
    }

    if(_base_ctrl == 1)
        twistInBase();

    if(joy->buttons[0]){
        _local_ctrl *= -1;
        if(_local_ctrl == 1)
            ROS_INFO("              \n    LOCAL ctrl");
        else
            ROS_INFO("              \n    GLOBAL ctrl");
    }
    if(_local_ctrl == 1)
        localCtrl();


    _desired_twist.twist.linear.x = _twist[0];
    _desired_twist.twist.linear.y = _twist[1];
    _desired_twist.twist.linear.z = _twist[2];
    _desired_twist.twist.angular.x = _twist[3];
    _desired_twist.twist.angular.y = _twist[4];
    _desired_twist.twist.angular.z = _twist[5];

    if(joy->buttons[1])
        activateDeactivateTask();

}

void JoyStick::twistInBase()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);

    std::string _task_base_link = srv.response.base_link;
    if(_task_base_link == "world")
        _task_base_link = "world_odom";

    try{
        ros::Time now = ros::Time::now();
        _listener.waitForTransform(_tf_prefix+_task_base_link,
                                   _tf_prefix+_robot_base_link,ros::Time(0),ros::Duration(1.0));

        _listener.lookupTransform(_tf_prefix+_task_base_link, _tf_prefix+"base_link",
            ros::Time(0), _transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    Eigen::Affine3d T;
    tf::transformTFToEigen(_transform, T);
    Eigen::MatrixXd A(6,6);
    A<<T.linear(),Eigen::MatrixXd::Zero(3,3),
       Eigen::MatrixXd::Zero(3,3),T.linear();

    _twist = A*_twist;
}

void JoyStick::sendVelRefs()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);
    
    if(srv.response.control_mode.compare("Disabled") != 0 && 
	srv.response.control_mode.compare("Velocity") == 1)
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
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);

    std::string _base_link = srv.response.base_link;
    if(_base_link == "world")
        _base_link = "world_odom";

    try{
        ros::Time now = ros::Time::now();
        _listener.waitForTransform(_tf_prefix+_base_link,
                                   _tf_prefix+_distal_links[_selected_task],ros::Time(0),ros::Duration(1.0));

        _listener.lookupTransform(_tf_prefix+_base_link, _tf_prefix+_distal_links[_selected_task],
            ros::Time(0), _transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    Eigen::Affine3d T;
    tf::transformTFToEigen(_transform, T);
    Eigen::MatrixXd A(6,6);
    A<<T.linear(),Eigen::MatrixXd::Zero(3,3),
       Eigen::MatrixXd::Zero(3,3),T.linear();

    _twist = A*_twist;
}

void JoyStick::activateDeactivateTask()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);

    std_srvs::SetBool srv2;

    if(srv.response.control_mode.compare("Disabled") == 0){
        srv2.request.data = true;
        ROS_INFO("              \n    ENABLING task");}
    else{
        srv2.request.data = false;
        ROS_INFO("              \n    DISABLING task");}

    _task_active_service_client[_selected_task].call(srv2);
}

std::string JoyStick::getRobotBaseLinkCtrlFrame()
{
    return _robot_base_link;
}

void JoyStick::setRobotBaseLinkCtrlFrame(const std::string& robot_base_link)
{
    _robot_base_link = robot_base_link;
}


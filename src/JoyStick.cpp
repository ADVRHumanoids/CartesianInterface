#include <cartesian_interface/joystick/JoyStick.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskInfo.h>


using namespace XBot::Cartesian;

JoyStick::JoyStick(const std::vector<std::string> &distal_links):
    //_nh("xbotcore/cartesian"),
    _nh(),
    _distal_links(distal_links),
    _selected_task(0),
    _linear_speed_sf(0.1),
    _angular_speed_sf(0.1)
{
    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStick::joyCallback, this);


    ros::ServiceClient srv_client;
    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        srv_client = _nh.serviceClient<cartesian_interface::SetTaskInfo>("xbotcore/cartesian/" + _distal_links[i] + "/set_task_properties");
        _set_properties_service_clients.push_back(srv_client);
    }

    ros::Publisher ref_pose_pub;
    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        std::string topic_name = "xbotcore/cartesian/" + _distal_links[i] + "/velocity_reference";
        ref_pose_pub = _nh.advertise<geometry_msgs::TwistStamped>(topic_name, 1);
        _ref_pose_pubs.push_back(ref_pose_pub);
    }

    ROS_INFO("Selected Task \n    distal_link: %s",_distal_links[_selected_task].c_str());
}

JoyStick::~JoyStick()
{

}

void JoyStick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[4])
    {
        _selected_task--;
        if(_selected_task  < 0)
            _selected_task = 0;
    }

    if(joy->buttons[5])
    {
        _selected_task++;
        if(_selected_task == _distal_links.size())
            _selected_task = _distal_links.size()-1;
    }

    if(joy->buttons[4] || joy->buttons[5])
        ROS_INFO("Selected Task \n    distal_link: %s",_distal_links[_selected_task].c_str());

    if(joy->buttons[7])
        setVelocityCtrl();



    _desired_twist.twist.linear.x = _linear_speed_sf * joy->axes[1];
    _desired_twist.twist.linear.y = _linear_speed_sf * joy->axes[0];
    _desired_twist.twist.linear.z = _linear_speed_sf * joy->axes[7];
    _desired_twist.twist.angular.x = _angular_speed_sf * joy->axes[6];
    _desired_twist.twist.angular.y = _angular_speed_sf * joy->axes[4];
    _desired_twist.twist.angular.z = _angular_speed_sf * joy->axes[3];

}

void JoyStick::sendVelRefs()
{
    _ref_pose_pubs[_selected_task].publish(_desired_twist);
}

void JoyStick::setVelocityCtrl()
{
    cartesian_interface::SetTaskInfo srv;

    srv.request.control_mode = "Velocity";
    _set_properties_service_clients[_selected_task].call(srv);

}


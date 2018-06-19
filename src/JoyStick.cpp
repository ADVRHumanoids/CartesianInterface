#include <cartesian_interface/joystick/JoyStick.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Transform.h>


using namespace XBot::Cartesian;

JoyStick::JoyStick(const std::vector<std::string> &distal_links, std::string tf_prefix):
    _nh("xbotcore/cartesian"),
    _tf_prefix(tf_prefix),
    _distal_links(distal_links),
    _selected_task(0),
    _linear_speed_sf(0.1),
    _angular_speed_sf(0.1),
    _twist(6), _local_ctrl(-1)
{
    _twist.setZero(6);
    _joy_sub = _nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyStick::joyCallback, this);


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

    ros::Publisher ref_pose_pub;
    for(unsigned int i = 0; i < distal_links.size(); ++i)
    {
        std::string topic_name = _distal_links[i] + "/velocity_reference";
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
        _selected_task--;

    if(joy->buttons[5])
        _selected_task++;

    _selected_task = _selected_task%_distal_links.size();

    if(joy->buttons[4] || joy->buttons[5])
        ROS_INFO("Selected Task \n    distal_link: %s",_distal_links[_selected_task].c_str());

    if(joy->buttons[7])
        setVelocityCtrl();


    _twist.setZero(6);
    _twist[0] = _linear_speed_sf * joy->axes[1];
    _twist[1] = _linear_speed_sf * joy->axes[0];
    _twist[2] = _linear_speed_sf * joy->axes[7];
    _twist[3] = _angular_speed_sf * joy->axes[6];
    _twist[4] = _angular_speed_sf * joy->axes[4];
    _twist[5] = _angular_speed_sf * joy->axes[3];

    if(joy->buttons[0])
        _local_ctrl *= -1;
    if(_local_ctrl == 1)
        localCtrl();


    _desired_twist.twist.linear.x = _twist[0];
    _desired_twist.twist.linear.y = _twist[1];
    _desired_twist.twist.linear.z = _twist[2];
    _desired_twist.twist.angular.x = _twist[3];
    _desired_twist.twist.angular.y = _twist[4];
    _desired_twist.twist.angular.z = _twist[5];

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

void JoyStick::localCtrl()
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_clients[_selected_task].call(srv);

    std::string _base_link = srv.response.base_link;

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


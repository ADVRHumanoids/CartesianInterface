#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/joystick/JoyStick.h>
#include <ros/ros.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>

using namespace XBot::Cartesian;

void constructJoyStick(ros::NodeHandle nh, JoyStick::Ptr& joystick)
{
    /* Get task list from cartesian server */
    ros::ServiceClient task_list_client = nh.serviceClient<cartesian_interface::GetTaskListRequest, cartesian_interface::GetTaskListResponse>("/xbotcore/cartesian/get_task_list");
    cartesian_interface::GetTaskListRequest req;
    cartesian_interface::GetTaskListResponse res;
    task_list_client.waitForExistence();
    if(!task_list_client.call(req, res))
        throw std::runtime_error("Unable to call /xbotcore/cartesian/get_task_list");

    std::vector<std::string> base_links;
    std::vector<std::string> distal_links;

    for(int i = 0; i < res.distal_links.size(); i++)
    {
        std::string ee_name = res.distal_links[i].data;
        std::string base_link = res.base_links[i].data;


        base_link = base_link == "world" ? "world_odom" : base_link;

        base_links.push_back(base_link);
        distal_links.push_back(ee_name);

    }

    joystick = boost::make_shared<XBot::Cartesian::JoyStick>(distal_links, base_links,"ci/");

    std::string robot_base_link;
    if(nh.getParam("robot_base_link", robot_base_link))
    {
        ROS_INFO("Got robot_base_link param: %s", robot_base_link.c_str());
        joystick->setRobotBaseLinkCtrlFrame(robot_base_link);
    }
}

int main(int argc, char **argv){

    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);

    /* Init ROS node */
    ros::init(argc, argv, "xbot_joystick_spawner");
    ros::NodeHandle nh("xbotcore/cartesian");

    JoyStick::Ptr joystick;
    constructJoyStick(nh, joystick);



    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        joystick->sendVelRefs();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}

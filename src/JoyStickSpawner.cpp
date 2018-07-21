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

int main(int argc, char **argv){

    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);

    /* Init ROS node */
    ros::init(argc, argv, "xbot_joystick_spawner");
    ros::NodeHandle nh("xbotcore/cartesian");

    /* Get task list from cartesian server */
    ros::ServiceClient task_list_client = nh.serviceClient<cartesian_interface::GetTaskListRequest, cartesian_interface::GetTaskListResponse>("/xbotcore/cartesian/get_task_list");
    cartesian_interface::GetTaskListRequest req;
    cartesian_interface::GetTaskListResponse res;
    task_list_client.waitForExistence();
    if(!task_list_client.call(req, res))
    {
        ros::shutdown();
        std::exit(1);
    }




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

    auto joystick = std::make_shared<XBot::Cartesian::JoyStick>(distal_links, base_links,"ci/");

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        joystick->sendVelRefs();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
}

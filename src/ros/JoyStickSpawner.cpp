#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/ToggleAxis.h>
#include <cartesian_interface/joystick/JoyStick.h>
#include <ros/ros.h>

#include <std_srvs/Trigger.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>

using namespace XBot::Cartesian;

std::shared_ptr<RosImpl> ci_ros;
std::unique_ptr<JoyStick> joystick;

void constructJoyStick(ros::NodeHandle nh, std::unique_ptr<JoyStick>& joystick)
{
    joystick.reset();

    ci_ros = std::make_shared<RosImpl>();

    ros::NodeHandle nh_priv("~");
    auto tf_prefix = nh_priv.param<std::string>("tf_prefix", "ci");
    if(tf_prefix == "null")
    {
        tf_prefix = "";
    }

    joystick.reset(new XBot::Cartesian::JoyStick(ci_ros, tf_prefix));

    std::string robot_base_link;
    if(nh.getParam("robot_base_link", robot_base_link))
    {
        ROS_INFO("Got robot_base_link param: %s", robot_base_link.c_str());
        joystick->setRobotBaseLinkCtrlFrame(robot_base_link);
    }
}

bool reset_callback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res, ros::NodeHandle nh)
{
    constructJoyStick(nh, joystick);

    res.message = "Successfully reset markers";
    res.success = true;

    return true;
}

bool toggle_axis_cbk(cartesian_interface::ToggleAxisRequest& req, 
                     cartesian_interface::ToggleAxisResponse& res)
{
    static const std::vector<std::string> field_names = {"TX", "TY", "TZ", "RX", "RY", "RZ"};
    
    res.message = "Succesfully set axis mask to\n";
    
    Eigen::Vector6i mask;
    for(int i = 0; i < 6; i++)
    {
        mask[i] = req.axis_mask[i];
        res.message += "  " + field_names[i] + ": " + (req.axis_mask[i] ? "enabled" : "disabled");
    }
    
    joystick->setTwistMask(mask);
    
    res.success = true;
    
    return true;
    
}

void controller_changed_callback(const std_msgs::EmptyConstPtr& msg, ros::NodeHandle nh)
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;

    if(reset_callback(req, res, nh) && res.success)
    {
        Logger::success(Logger::Severity::HIGH, "Reset joystick succesfully\n");
    }
    else
    {
        Logger::error(Logger::Severity::HIGH, "Failed to reset joystick\n");
    }
}

int main(int argc, char **argv){

    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);

    /* Init ROS node */
    ros::init(argc, argv, "joystick_spawner");
    ros::NodeHandle nh("cartesian");

    /* Reset service */
    auto srv_cbk = std::bind(reset_callback, std::placeholders::_1, std::placeholders::_2, nh);
    auto srv = nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("joystick/reset",
                                                                                        srv_cbk);
    
    /* Toggle axis service */
    auto toggle_axis_srv = nh.advertiseService("joystick/toggle_axis",
                                               toggle_axis_cbk);

    /* Controller changed subscriber */
    auto event_sub_cbk = std::bind(controller_changed_callback, std::placeholders::_1, nh);
    auto event_sub = nh.subscribe<std_msgs::Empty>("changed_controller_event", 1, event_sub_cbk);


    constructJoyStick(nh, joystick);

    ros::Rate loop_rate(30.0);
    
    while(ros::ok())
    {
        ros::spinOnce();
        joystick->updateStatus();
        joystick->sendVelRefs();
//        joystick->broadcastStatus();
        loop_rate.sleep();
    }
    
    joystick.reset();
}

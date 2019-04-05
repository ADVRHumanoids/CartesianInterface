#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <std_srvs/Trigger.h>
#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/markers/CartesianMarker.h>

#include <functional>

#include <ros/ros.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>



using namespace XBot::Cartesian;
typedef std::map<std::string, CartesianMarker::Ptr> MarkerMap;

MarkerMap g_markers;
std::string g_tf_prefix_slash;

void construct_markers(ros::NodeHandle nh, MarkerMap& markers);
bool reset_callback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res, ros::NodeHandle nh);
void controller_changed_callback(const std_msgs::EmptyConstPtr& msg, ros::NodeHandle nh);

int main(int argc, char **argv){
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "xbot_cartesian_marker_spawner");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");
    
    /* Reset service */
    auto srv_cbk = std::bind(reset_callback, std::placeholders::_1, std::placeholders::_2, nh);
    auto srv = nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("markers/reset", srv_cbk);
    
    /* Controller changed subscriber */
    auto event_sub_cbk = std::bind(controller_changed_callback, std::placeholders::_1, nh);
    auto event_sub = nh.subscribe<std_msgs::Empty>("changed_controller_event", 1, event_sub_cbk);
    
    /* TF prefix from param */
    g_tf_prefix_slash = nh_priv.param<std::string>("tf_prefix", "ci");
    if(g_tf_prefix_slash == "null")
    {
        g_tf_prefix_slash = "";
    }
    g_tf_prefix_slash = g_tf_prefix_slash == "" ? "" : (g_tf_prefix_slash + "/");
    
    construct_markers(nh, g_markers);
    
    Logger::success(Logger::Severity::HIGH, "Marker spawner: started spinning...\n");
    ros::spin();
    
    g_markers.clear();
}

void controller_changed_callback(const std_msgs::EmptyConstPtr& msg, ros::NodeHandle nh)
{
    std_srvs::TriggerRequest req;
    std_srvs::TriggerResponse res;
    
    if(reset_callback(req, res, nh) && res.success)
    {
        Logger::success(Logger::Severity::HIGH, "Reset markers succesfully\n");
    }
    else
    {
        Logger::error(Logger::Severity::HIGH, "Failed to reset markers\n");
    }
}


bool reset_callback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res, ros::NodeHandle nh)
{
    g_markers.clear();
    
    construct_markers(nh, g_markers); // will THROW on failure
    
    res.message = "Successfully reset markers";
    res.success = true;
    
    return true;
}

void construct_markers(ros::NodeHandle nh, MarkerMap& markers)
{
    /* Get task list from cartesian server */
    ros::ServiceClient task_list_client = nh.serviceClient<cartesian_interface::GetTaskList>("get_task_list");
    cartesian_interface::GetTaskListRequest req;
    cartesian_interface::GetTaskListResponse res;
    task_list_client.waitForExistence();
    Logger::info(Logger::Severity::HIGH, "Marker spawner: retrieving task list\n");
    if(!task_list_client.call(req, res))
    {
        throw std::runtime_error("Unable to call get_task_list service");
    }
    
    std::string robot_urdf_string;
    nh.getParam("/robot_description", robot_urdf_string);
    urdf::Model robot_urdf;
    robot_urdf.initString(robot_urdf_string);


    for(int i = 0; i < res.distal_links.size(); i++)
    {
        std::string ee_name = res.distal_links[i];
        std::string base_link = res.base_links[i];
        unsigned int control_type;
        bool use_mesh = true;
        
        if(ee_name == "com" || ee_name == "gaze")
        {
            control_type = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
            use_mesh = false;
        }
        else
        {
            control_type = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }
        
        base_link = base_link == "world" ? "world_odom" : base_link;
        auto marker = std::make_shared<CartesianMarker>(base_link,
                                                   ee_name,
                                                   robot_urdf,
                                                   control_type,
                                                   g_tf_prefix_slash,
                                                   use_mesh
                                                  );
        
        markers[ee_name] = marker;
    }
    

    
   
}

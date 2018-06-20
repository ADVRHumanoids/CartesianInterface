#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <XBotInterface/SoLib.h>

#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/markers/CartesianMarker.h>

#include <ros/ros.h>

#define BOOST_RESULT_OF_USE_DECLTYPE
#include <boost/function.hpp>
#include <chrono>



using namespace XBot::Cartesian;


int main(int argc, char **argv){
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    ros::init(argc, argv, "marker_spawner");
    ros::NodeHandle nh;

    /* Get task list from cartesian server */
    ros::ServiceClient task_list_client = nh.serviceClient<cartesian_interface::GetTaskListRequest, cartesian_interface::GetTaskListResponse>("/get_task_list");
    cartesian_interface::GetTaskListRequest req;
    cartesian_interface::GetTaskListResponse res;
    task_list_client.waitForExistence();
    if(!task_list_client.call(req, res))
    {
        ros::shutdown();
        std::exit(1);
    }
    
    std::string robot_urdf_string;
    nh.getParam("/robot_description", robot_urdf_string);
    urdf::Model robot_urdf;
    robot_urdf.initString(robot_urdf_string);
    
    
    std::map<std::string, XBot::Cartesian::CartesianMarker::Ptr> markers;

    for(int i = 0; i < res.distal_links.size(); i++)
    {
        std::string ee_name = res.distal_links[i].data;
        std::string base_link = res.base_links[i].data;
        unsigned int control_type;
        
        if(ee_name == "com")
        {
            control_type = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
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
                                                   "ci/"
                                                  );
        
        markers[ee_name] = marker;
    }
    
    ros::spin();
}

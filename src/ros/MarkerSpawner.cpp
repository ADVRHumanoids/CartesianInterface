#include <xbot2_interface/logger.h>

#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <cartesian_interface/srv/get_task_list.hpp>
#include <cartesian_interface/markers/CartesianMarker.h>

#include <rclcpp/wait_for_message.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace XBot::Cartesian;

using namespace std::chrono_literals;

using XBot::Logger;

typedef std::map<std::string, CartesianMarker::Ptr> MarkerMap;

MarkerMap g_markers;

std::string g_tf_prefix_slash;

rclcpp::Node::SharedPtr g_node;

urdf::Model g_robot_urdf;

void construct_markers();

bool reset_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res);

void controller_changed_callback(std_msgs::msg::Empty::ConstSharedPtr msg);

int main(int argc, char **argv){
    
    /* By default, MID verbosity level */
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    /* Init ROS node */
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("marker_spawner_node");

    std::string ns = node->declare_parameter<std::string>("namespace", "cartesian");

    auto sub_node = node->create_sub_node(ns);

    g_node = sub_node;

    /* Urdf model */
    std_msgs::msg::String urdf_string;

    auto urdf_topic_name = "robot_description";

    while(!rclcpp::wait_for_message(urdf_string,
                             g_node,
                             urdf_topic_name,
                                     1s))
    {
        RCLCPP_INFO_STREAM(g_node->get_logger(),
                           "waiting for urdf on topic "
                               << g_node->get_node_topics_interface()->resolve_topic_name(
                                      urdf_topic_name));
    }

    if(!g_robot_urdf.initString(urdf_string.data))
    {
        RCLCPP_ERROR(g_node->get_logger(),
                     "could not init urdf model");
    }

    /* Reset service */
    auto srv = sub_node->create_service<std_srvs::srv::Trigger>("markers/reset", reset_callback);
    
    /* Controller changed subscriber */
    auto event_sub = sub_node->create_subscription<std_msgs::msg::Empty>("changed_controller_event",
                                                                         1,
                                                                         controller_changed_callback);

    /* TF prefix from param */
    g_tf_prefix_slash = sub_node->declare_parameter<std::string>("tf_prefix", "ci");

    if(g_tf_prefix_slash == "null")
    {
        g_tf_prefix_slash = "";
    }

    g_tf_prefix_slash = g_tf_prefix_slash == "" ? "" : (g_tf_prefix_slash + "/");


    /* Consruct marker and spin */
    construct_markers();
    
    Logger::success(Logger::Severity::HIGH, "Marker spawner: started spinning...\n");

    rclcpp::spin(g_node);

    g_node.reset();
    
    g_markers.clear();
}

void controller_changed_callback(std_msgs::msg::Empty::ConstSharedPtr msg)
{
    if(reset_callback(nullptr, nullptr))
    {
        Logger::success(Logger::Severity::HIGH, "reset markers succesfully \n");
    }
    else
    {
        Logger::error(Logger::Severity::HIGH, "failed to reset markers \n");
    }
}

bool reset_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                    std_srvs::srv::Trigger::Response::SharedPtr res)
{
    g_markers.clear();
    
    construct_markers(); // will throw on failure
    
    if(res)
    {
        res->message = "successfully reset markers";
        res->success = true;
    }

    return true;
}

void construct_markers()
{
    // clear previously built markers if any
    g_markers.clear();

    // get task list from cartesian server
    auto task_list_client = g_node->create_client<cartesian_interface::srv::GetTaskList>("get_task_list");
    while(!task_list_client->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(g_node->get_logger(),
                           "waiting for server " << task_list_client->get_service_name()
                           );
    }

    auto req = std::make_shared<cartesian_interface::srv::GetTaskList::Request>();

    auto fut = task_list_client->async_send_request(req);

    if(rclcpp::spin_until_future_complete(g_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {

    }
    else
    {
        throw std::runtime_error("unable to call get_task_list service");
    }

    const auto& res = *fut.get();

    for(int i = 0; i < res.names.size(); i++)
    {
        unsigned int control_type;

        bool use_mesh = true;
        
        if(res.types[i] == "Com" || res.types[i] == "Gaze")
        {
            control_type = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
            use_mesh = false;
        }
        else if(res.types[i] == "Cartesian" || res.types[i] == "Interaction")
        {
            control_type = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }
        else
        {
            continue;
        }
        
        auto marker = std::make_shared<CartesianMarker>(res.names[i],
                                                        g_robot_urdf,
                                                        control_type,
                                                        g_node,
                                                        g_tf_prefix_slash,
                                                        use_mesh
                                                        );
        
        g_markers[res.names[i]] = marker;
    }
    

    

}


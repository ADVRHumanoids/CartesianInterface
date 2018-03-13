#include <ros/ros.h>
#include <CartesianInterface/markers/CartesianMarker.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CartesianMarkerExample_node");

    ros::NodeHandle nh("~");

    //Getting parameters from param server
    std::string base_link;
    nh.getParam("base_link", base_link);
    std::string distal_link;
    nh.getParam("distal_link", distal_link);
    std::string robot_model;
    nh.getParam("/robot_description", robot_model);
    urdf::Model robot_urdf;
    robot_urdf.initString(robot_model);

    XBot::Cartesian::CartesianMarker marker(base_link, distal_link, robot_urdf);

    ROS_INFO("Running CartesianMarkerExample_node");
    ros::spin();
}

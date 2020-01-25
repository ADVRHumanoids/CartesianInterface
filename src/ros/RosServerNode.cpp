#include <cartesian_interface/ros/RosExecutor.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ros_server_node");
    
    RosExecutor exec;
    
    exec.spin();

}

#include <cartesian_interface/ros/RosExecutor.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    RosExecutor exec;
    
    exec.spin();

}

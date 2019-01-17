#include <cartesian_interface/problem/Cartesian.h>

using namespace XBot::Cartesian;

CartesianTask::CartesianTask(std::string distal_link, std::string base_link, int size, std::string type):
    TaskDescription(TaskInterface::Cartesian, type, size),
    distal_link(distal_link),
    base_link(base_link),
    orientation_gain(1.0)
{

}

CartesianTask::Ptr XBot::Cartesian::MakeCartesian(std::string distal_link, std::string base_link)
{
    return std::make_shared<CartesianTask>(distal_link, base_link);
}

CartesianTask::Ptr XBot::Cartesian::GetAsCartesian(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<CartesianTask>(task);
}

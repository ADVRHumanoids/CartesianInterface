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

TaskDescription::Ptr CartesianTask::yaml_parse_cartesian(YAML::Node task_node, ModelInterface::ConstPtr model)
{

    std::string distal_link = task_node["distal_link"].as<std::string>();
    std::string base_link = "world";


    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }

    auto cart_task = MakeCartesian(distal_link, base_link);
    TaskDescription::Ptr task_desc = cart_task;


    if(task_node["orientation_gain"])
    {
        cart_task->orientation_gain = task_node["orientation_gain"].as<double>();
    }

    if(task_node["body_jacobian"])
    {
        cart_task->is_body_jacobian = task_node["body_jacobian"].as<bool>();
    }

    return task_desc;
}

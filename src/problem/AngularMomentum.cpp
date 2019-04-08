#include <cartesian_interface/problem/AngularMomentum.h>

using namespace XBot::Cartesian;


AngularMomentumTask::Ptr XBot::Cartesian::MakeAngularMomentum()
{
    return std::make_shared<AngularMomentumTask>();
}

AngularMomentumTask::Ptr XBot::Cartesian::GetAsAngularMomentum(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<AngularMomentumTask>(task);
}

XBot::Cartesian::AngularMomentumTask::AngularMomentumTask():
    TaskDescription(TaskInterface::None, "AngularMomentum", 3),
    min_rate(false)
{

}

TaskDescription::Ptr AngularMomentumTask::yaml_parse_angular_momentum(YAML::Node node, ModelInterface::ConstPtr model)
{
    auto task_desc = MakeAngularMomentum();
    auto angularmom_desc = GetAsAngularMomentum(task_desc);

    if(node["min_rate"] && node["min_rate"].as<bool>())
    {
        angularmom_desc->min_rate = true;
    }

    return task_desc;
}


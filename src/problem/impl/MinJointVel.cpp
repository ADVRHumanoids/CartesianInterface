#include <cartesian_interface/problem/MinJointVel.h>

using namespace XBot::Cartesian;

MinJointVelTask::Ptr XBot::Cartesian::GetAsMinJointVel(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<MinJointVelTask>(task);
}

MinJointVelTask::MinJointVelTask(int ndof):
    TaskDescription(TaskInterface::None, "MinJointVel", ndof),
    eps(1.)
{

}

MinJointVelTask::Ptr XBot::Cartesian::MakeMinJointVel(int ndof)
{
    return std::make_shared<MinJointVelTask>(ndof);
}

TaskDescription::Ptr MinJointVelTask::yaml_parse_minjointvel(YAML::Node task_node,
                                                   ModelInterface::ConstPtr model)
{
    auto task_desc = MakeMinJointVel(model->getJointNum());
    auto postural_desc = GetAsMinJointVel(task_desc);

    if(task_node["eps"] && task_node["eps"].as<double>())
    {
        postural_desc->eps = task_node["eps"].as<double>();
    }

    return task_desc;
}

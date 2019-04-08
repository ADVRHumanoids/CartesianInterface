#include <cartesian_interface/problem/Limits.h>

using namespace XBot::Cartesian;

ConstraintDescription::Ptr XBot::Cartesian::MakeJointLimits()
{
    return std::make_shared<ConstraintDescription>("JointLimits");
}

ConstraintDescription::Ptr XBot::Cartesian::MakeVelocityLimits()
{
    return std::make_shared<ConstraintDescription>("VelocityLimits");
}


ConstraintDescription::ConstraintDescription(std::string __type):
    type(__type)
{

}


ConstraintFromTask::ConstraintFromTask(TaskDescription::Ptr arg_task):
    ConstraintDescription("ConstraintFromTask"),
    task(arg_task)
{

}


XBot::Cartesian::ConstraintDescription::Ptr XBot::Cartesian::MakeConstraintFromTask(XBot::Cartesian::TaskDescription::Ptr task)
{
    return std::make_shared<ConstraintFromTask>(task);
}


TaskDescription::Ptr XBot::Cartesian::GetTaskFromConstraint(ConstraintDescription::Ptr constr)
{
    auto constr_from_task = std::dynamic_pointer_cast<ConstraintFromTask>(constr);
    
    if(constr_from_task)
    {
        return constr_from_task->task;
    }
    else
    {
        return nullptr;
    }
}

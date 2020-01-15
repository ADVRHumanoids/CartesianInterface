#include <cartesian_interface/problem/Limits.h>

using namespace XBot::Cartesian;


ConstraintFromTask::ConstraintFromTask(TaskDescription::Ptr arg_task):
    ConstraintDescription("ConstraintFromTask",
                          arg_task->getName(),
                          arg_task->getSize(),
                          arg_task->getModel()),
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


bool XBot::Cartesian::ConstraintFromTask::validate()
{
    return task->validate();
}

void XBot::Cartesian::ConstraintFromTask::update(double time, double period)
{
    task->update(time, period);
}

void XBot::Cartesian::ConstraintFromTask::reset()
{
    task->reset();
}

void XBot::Cartesian::ConstraintFromTask::log(MatLogger::Ptr logger, bool init_logger, int buf_size)
{
    task->log(logger, init_logger, buf_size);
}

bool ConstraintDescription::IsConstraint(TaskDescription::ConstPtr task)
{
    return static_cast<bool>(AsConstraint(task));
}

ConstraintDescription::Ptr ConstraintDescription::AsConstraint(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<ConstraintDescription>(task);
}

ConstraintDescription::ConstPtr ConstraintDescription::AsConstraint(TaskDescription::ConstPtr task)
{
    return std::dynamic_pointer_cast<const ConstraintDescription>(task);
}

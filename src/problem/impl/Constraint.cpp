#include "problem/Constraint.h"

using namespace XBot::Cartesian;


ConstraintFromTaskImpl::ConstraintFromTaskImpl(TaskDescriptionImpl::Ptr arg_task):
    TaskDescriptionImpl("ConstraintFromTask",
                          arg_task->getName(),
                          arg_task->getSize(),
                          arg_task->getContext()),
    _task(arg_task)
{

}


ConstraintDescription::Ptr XBot::Cartesian::MakeConstraintFromTask(XBot::Cartesian::TaskDescription::Ptr task)
{
    if(auto task_impl = std::dynamic_pointer_cast<TaskDescriptionImpl>(task))
    {
        return std::make_shared<ConstraintFromTaskImpl>(task_impl);
    }

    throw std::invalid_argument("MakeConstraintFromTask needs a TaskDescriptionImpl object as argument");
}


TaskDescription::Ptr XBot::Cartesian::GetTaskFromConstraint(ConstraintDescription::Ptr constr)
{
    auto constr_from_task = std::dynamic_pointer_cast<ConstraintFromTask>(constr);
    
    if(constr_from_task)
    {
        return constr_from_task->getTask();
    }
    else
    {
        return nullptr;
    }
}


bool XBot::Cartesian::ConstraintFromTaskImpl::validate()
{
    return _task->validate();
}

void XBot::Cartesian::ConstraintFromTaskImpl::update(double time, double period)
{
    _task->update(time, period);
}

void XBot::Cartesian::ConstraintFromTaskImpl::reset()
{
    _task->reset();
}

void XBot::Cartesian::ConstraintFromTaskImpl::log(MatLogger2::Ptr logger, bool init_logger, int buf_size)
{
    _task->log(logger, init_logger, buf_size);
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


TaskDescription::Ptr XBot::Cartesian::ConstraintFromTaskImpl::getTask()
{
    return _task;
}


bool XBot::Cartesian::ConstraintFromTaskImpl::getTaskError(Eigen::VectorXd &e) const
{
    return _task->getTaskError(e);
}

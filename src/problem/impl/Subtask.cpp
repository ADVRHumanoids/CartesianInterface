#include "Subtask.h"

using namespace XBot::Cartesian;


SubtaskImpl::SubtaskImpl(TaskDescriptionImpl::Ptr task,
                         std::vector<int> indices,
                         std::string subtask_name):
    TaskDescriptionImpl("Subtask",
                        subtask_name == "" ? gen_subtask_name(task, indices) : subtask_name,
                        task->getSize(),
                        task->getModel()),
    _task(task)
{
    setIndices(indices);
}

std::string SubtaskImpl::gen_subtask_name(TaskDescriptionImpl::Ptr task, std::vector<int> indices)
{
    std::stringstream ss;
    ss << task->getName() << "_subtask";

    for(auto i : indices)
    {
        ss << "_" << i;
    }

    return ss.str();
}


TaskDescription::Ptr XBot::Cartesian::SubtaskImpl::getTask()
{
    return _task;
}



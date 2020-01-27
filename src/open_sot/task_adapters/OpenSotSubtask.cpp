#include "OpenSotSubtask.h"

using namespace XBot::Cartesian;

OpenSotSubtaskAdapter::OpenSotSubtaskAdapter(TaskDescription::Ptr task,
                                             XBot::ModelInterface::ConstPtr model):
    OpenSotTaskAdapter(task, model)
{
    _ci_subtask = std::dynamic_pointer_cast<Subtask>(task);

    if(!_ci_subtask) throw std::runtime_error("Provided task description "
                                              "does not have expected type 'Subtask'");

    try
    {
        _task_adapter = OpenSotTaskAdapter::MakeInstance(_ci_subtask->getTask(),
                                                         model);
    }
    catch(...)
    {
        fprintf(stderr, "Unable to construct task from subtask \n");
        throw;
    }

}

TaskPtr OpenSotSubtaskAdapter::constructTask()
{
    return _task_adapter->getOpenSotTask();
}

bool OpenSotSubtaskAdapter::initialize()
{
    return OpenSotTaskAdapter::initialize();
}

void OpenSotSubtaskAdapter::update(double time, double period)
{
    return _task_adapter->update(time, period);
}

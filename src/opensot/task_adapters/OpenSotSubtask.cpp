#include "opensot/OpenSotSubtask.h"

using namespace XBot::Cartesian;

OpenSotSubtaskAdapter::OpenSotSubtaskAdapter(TaskDescription::Ptr task,
                                             Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_subtask = std::dynamic_pointer_cast<Subtask>(task);

    if(!_ci_subtask) throw std::runtime_error("Provided task description "
                                              "does not have expected type 'Subtask'");

    try
    {
        _task_adapter = OpenSotTaskAdapter::MakeInstance(_ci_subtask->getTask(),
                                                         context);
    }
    catch(...)
    {
        fprintf(stderr, "Unable to construct task from subtask \n");
        throw;
    }

}

TaskPtr OpenSotSubtaskAdapter::constructTask()
{
    std::list<uint> indices_list(_ci_subtask->getSubtaskIndices().begin(),
                                 _ci_subtask->getSubtaskIndices().end());

    return std::make_shared<SubtaskSoT>(_task_adapter->getOpenSotTask(), indices_list);
}

bool OpenSotSubtaskAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return _task_adapter->initialize(vars) &&
                OpenSotTaskAdapter::initialize(vars);
}

void OpenSotSubtaskAdapter::update(double time, double period)
{
    return _task_adapter->update(time, period);
}


OpenSoT::OptvarHelper::VariableVector OpenSotSubtaskAdapter::getRequiredVariables() const
{
    return _task_adapter->getRequiredVariables();
}

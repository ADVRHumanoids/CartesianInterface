#include "OpenSotConstraintFromTask.h"

using namespace XBot::Cartesian;

OpenSotConstraintFromTaskAdapter::OpenSotConstraintFromTaskAdapter(ConstraintDescription::Ptr constr,
                                                                   XBot::ModelInterface::ConstPtr model):
    OpenSotConstraintAdapter(constr, model)
{
    auto constr_from_task = std::dynamic_pointer_cast<ConstraintFromTask>(constr);

    if(!constr_from_task)
    {
        throw std::runtime_error("Provided task description "
                                 "does not have expected type 'ConstraintFromTask'");
    }

    // construct and initialize task adapter
    try
    {
        _task_adapter = OpenSotTaskAdapter::MakeInstance(constr_from_task->getTask(),
                                                         model);
    }
    catch(...)
    {
        fprintf(stderr, "Unable to construct task from constraint \n");
        throw;
    }

}

ConstraintPtr OpenSotConstraintFromTaskAdapter::constructConstraint()
{
    return boost::make_shared<OpenSoT::constraints::TaskToConstraint>(_task_adapter->getOpenSotTask());
}

bool OpenSotConstraintFromTaskAdapter::initialize()
{
    OpenSotConstraintAdapter::initialize();

    return true;
}

void OpenSotConstraintFromTaskAdapter::update(double time, double period)
{
    _task_adapter->update(time, period);
}


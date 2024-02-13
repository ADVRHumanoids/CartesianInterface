#include "OpenSotMinEffort.h"

#include <boost/make_shared.hpp>

#include "fmt/format.h"

using namespace XBot::Cartesian;

OpenSotMinimumEffort::OpenSotMinimumEffort(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_task = std::dynamic_pointer_cast<MinimumEffort>(task);

    if(!_ci_task) throw std::runtime_error("Provided task description "
                                             "does not have expected type 'MinimumEffort'");

}

TaskPtr OpenSotMinimumEffort::constructTask()
{

    _sot_task = SotUtils::make_shared<MinimumEffortSoT>(*_model,
                                                        _ci_task->getStepSize()
                                                        );

    return _sot_task;
}

bool OpenSotMinimumEffort::initialize(const ::OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotMinimumEffort] requires default variables definition");
    }

    return OpenSotTaskAdapter::initialize(vars);
}

void OpenSotMinimumEffort::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    // any custom update action
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotMinimumEffort, MinimumEffort)


#include "OpenSotManipulability.h"
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>

#include <boost/make_shared.hpp>

#include "fmt/format.h"

using namespace XBot::Cartesian;

OpenSotManipulability::OpenSotManipulability(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_task = std::dynamic_pointer_cast<Manipulability>(task);

    if(!_ci_task) throw std::runtime_error("Provided task description "
                                             "does not have expected type 'Manipulability'");

}

TaskPtr OpenSotManipulability::constructTask()
{
    std::string base_link = _ci_task->getBaseLink();
    std::string distal_link = _ci_task->getDistalLink();

    if(distal_link == "com")
    {
        _sot_task = std::make_shared<OpenSoT::tasks::velocity::CoM>(const_cast<ModelInterface&>(*_model));

        _sot_task_manip = SotUtils::make_shared<ManipulabilitySoT>(*_model,
                                                                   OpenSoT::tasks::velocity::CoM::asCoM(_sot_task),
                                                            _ci_task->getStepSize());
    }
    else
    {
        _sot_task = std::make_shared<OpenSoT::tasks::velocity::Cartesian>(distal_link, const_cast<ModelInterface&>(*_model), distal_link, base_link);
        _sot_task_manip = SotUtils::make_shared<ManipulabilitySoT>(*_model,
                                                                   OpenSoT::tasks::velocity::Cartesian::asCartesian(_sot_task),
                                                            _ci_task->getStepSize());
    }


    return _sot_task_manip;
}

bool OpenSotManipulability::initialize(const ::OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotManipulability] requires default variables definition");
    }

    return OpenSotTaskAdapter::initialize(vars);
}

void OpenSotManipulability::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    // any custom update action
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotManipulability, Manipulability)


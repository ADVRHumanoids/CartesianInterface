#include "OpenSotTask.h"

using namespace XBot::Cartesian;

OpenSotTaskAdapter::OpenSotTaskAdapter(TaskDescription::Ptr task,
                                       ModelInterface::ConstPtr model):
    _ci_task(task)
{

}

bool OpenSotTaskAdapter::initialize()
{
    _ci_task->registerObserver(shared_from_this());

    return true;
}

void OpenSotTaskAdapter::update(double time, double period)
{
    _opensot_task->setLambda(_ci_task->getLambda());
}

TaskPtr OpenSotTaskAdapter::getOpenSotTask()
{
    return _opensot_task;
}

bool OpenSotTaskAdapter::onWeightChanged()
{
    _opensot_task->setWeight(_ci_task->getWeight());
    return true;
}

bool OpenSotTaskAdapter::onActivationStateChanged()
{
    auto activ = _ci_task->getActivationState();

    if(activ == ActivationState::Disabled)
    {
        _opensot_task->setActive(false);
    }

    if(activ == ActivationState::Enabled)
    {
        _opensot_task->setActive(true);
    }

    return true;
}

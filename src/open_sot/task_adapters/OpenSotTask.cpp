#include "OpenSotTask.h"

#include <OpenSoT/utils/AutoStack.h>

#include "../../utils/DynamicLoading.h"

#include "OpenSotCartesian.h"
#include "OpenSotJointLimits.h"
#include "OpenSotConstraintFromTask.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;

OpenSotTaskAdapter::OpenSotTaskAdapter(TaskDescription::Ptr task,
                                       ModelInterface::ConstPtr model):
    _ci_task(task), _model(model)
{

}

bool OpenSotTaskAdapter::initialize()
{
    /* Let derived class construct the task */
    _sub_task = _opensot_task = constructTask();


    /* Set generic task parameters */

    // active joint mask
    if(_ci_task->getDisabledJoints().size() > 0)
    {
        std::vector<bool> active_joints_mask(_model->getJointNum(), true);

        for(auto jstr : _ci_task->getDisabledJoints())
        {
            active_joints_mask.at(_model->getDofIndex(jstr)) = false;
        }

        _opensot_task->setActiveJointsMask(active_joints_mask);
    }

    // lambda
    _opensot_task->setLambda(_ci_task->getLambda());

    // weight
    _opensot_task->setWeight(_ci_task->getWeight());

    // indices
    if(_ci_task->getIndices().size() != _ci_task->getSize())
    {
        std::list<uint> indices_list(_ci_task->getIndices().begin(),
                                     _ci_task->getIndices().end());

        _sub_task = _opensot_task % indices_list;
    }


    /* Register observer */
    _ci_task->registerObserver(shared_from_this());

    return true;
}

void OpenSotTaskAdapter::update(double time, double period)
{
    _opensot_task->setLambda(_ci_task->getLambda());
}

TaskPtr OpenSotTaskAdapter::getOpenSotTask()
{
    return _sub_task;
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

OpenSotTaskAdapter::Ptr OpenSotTaskAdapter::MakeInstance(TaskDescription::Ptr task,
                                                         XBot::ModelInterface::ConstPtr model)
{
    OpenSotTaskAdapter * task_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(task->getLibName() != "")
    {
        task_adapter = CallFunction<OpenSotTaskAdapter*>(task->getLibName(),
                                                                  "create_opensot_task_adapter",
                                                                  task, model);
    }
    else if(task->getType() == "Cartesian") /* Otherwise, construct supported tasks */
    {
        task_adapter = new OpenSotCartesianAdapter(task, model);
    }
    else if(task->getType() == "Postural")
    {

    }
    else
    {
        auto str = fmt::format("Unable to construct OpenSotTaskAdapter instance for task '{}': "
                               "empty lib name for unrecoginized task type '{}'",
                               task->getName(), task->getType());
        throw std::runtime_error(str);
    }

    if(!task_adapter->initialize())
    {
        throw std::runtime_error(fmt::format("Unable to inizialize task '{}'", task->getName()));
    }

    return Ptr(task_adapter);
}

OpenSotConstraintAdapter::OpenSotConstraintAdapter(ConstraintDescription::Ptr constr,
                                                   XBot::ModelInterface::ConstPtr model):
    _ci_constr(constr), _model(model)
{

}

bool OpenSotConstraintAdapter::initialize()
{
    /* Register observer */
    _ci_constr->registerObserver(shared_from_this());

    return true;
}

void OpenSotConstraintAdapter::update(double time, double period)
{

}

ConstraintPtr OpenSotConstraintAdapter::getOpenSotConstraint()
{
    return _opensot_constr;
}

OpenSotConstraintAdapter::Ptr OpenSotConstraintAdapter::MakeInstance(ConstraintDescription::Ptr constr,
                                                                     XBot::ModelInterface::ConstPtr model)
{
    OpenSotConstraintAdapter * constr_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(constr->getLibName() != "")
    {
        constr_adapter = CallFunction<OpenSotConstraintAdapter*>(constr->getLibName(),
                                                                 "create_opensot_constr_adapter",
                                                                 constr, model);
    }
    else if(constr->getType() == "ConstraintFromTask")
    {
        constr_adapter = new OpenSotConstraintFromTaskAdapter(constr, model);
    }
    else if(constr->getType() == "JointLimits") /* Otherwise, construct supported tasks */
    {
        constr_adapter = new OpenSotJointLimitsAdapter(constr, model);
    }
    else if(constr->getType() == "VelocityLimits")
    {

    }
    else
    {
        auto str = fmt::format("Unable to construct OpenSotConstraintAdapter instance for task '{}': "
                               "empty lib name for unrecoginized constraint type '{}'",
                               constr->getName(), constr->getType());
        throw std::runtime_error(str);
    }

    if(!constr_adapter->initialize())
    {
        throw std::runtime_error(fmt::format("Unable to inizialize constraint '{}'", constr->getName()));
    }

    return Ptr(constr_adapter);
}


#include "opensot/OpenSotTask.h"

#include <OpenSoT/utils/AutoStack.h>

#include "../../utils/DynamicLoading.h"

#include "opensot/OpenSotCartesian.h"
#include "opensot/OpenSotJointLimits.h"
#include "opensot/OpenSotVelocityLimits.h"
#include "opensot/OpenSotConstraintFromTask.h"
#include "opensot/OpenSotPostural.h"
#include "opensot/OpenSotCom.h"
#include "opensot/OpenSotSubtask.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;

OpenSotTaskAdapter::OpenSotTaskAdapter(TaskDescription::Ptr task,
                                       Context::ConstPtr context
                                       ):
    _vars({}),
    _model(context->model()),
    _ci_task(task),
    _ctx(context)
{

}

bool OpenSotTaskAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    /* Save variables */
    _vars = vars;

    /* Let derived class construct the task */
    _opensot_task = constructTask();
    _sub_task = _opensot_task;

    if(!_opensot_task)
    {
        throw std::runtime_error(fmt::format("OpenSotTaskAdapter: constructTask() for task '{}' returned null pointer",
                                             _ci_task->getName()));
    }


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

    // activation state
    auto activ = _ci_task->getActivationState();

    if(activ == ActivationState::Disabled)
    {
        _sub_task->setActive(false);
    }


    /* Register observer */
    _ci_task->registerObserver(shared_from_this());

    return true;
}

OpenSoT::OptvarHelper::VariableVector OpenSotTaskAdapter::getRequiredVariables() const
{
    return {};
}

void OpenSotTaskAdapter::update(double time, double period)
{
    _opensot_task->setLambda(_ci_task->getLambda());
}

TaskPtr OpenSotTaskAdapter::getOpenSotTask()
{
    return _sub_task;
}

TaskDescription::Ptr OpenSotTaskAdapter::getTaskDescription() const
{
    return _ci_task;
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
        _sub_task->setActive(false);
    }

    if(activ == ActivationState::Enabled)
    {
        _sub_task->setActive(true);
    }

    return true;
}

OpenSotTaskAdapter::Ptr OpenSotTaskAdapter::MakeInstance(TaskDescription::Ptr task,
                                                         Context::ConstPtr context)
{
    OpenSotTaskAdapter * task_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(task->getLibName() != "")
    {
        task_adapter = CallFunction<OpenSotTaskAdapter*>(task->getLibName(),
                                                         "create_opensot_" + task->getType() + "_adapter",
                                                         task,
                                                         context,
                                                         detail::Version CARTESIO_ABI_VERSION);
    }
    else if(task->getType() == "Cartesian") /* Otherwise, construct supported tasks */
    {
        task_adapter = new OpenSotCartesianAdapter(task, context);
    }
    else if(task->getType() == "Postural")
    {
        task_adapter = new OpenSotPosturalAdapter(task, context);
    }
    else if(task->getType() == "Com")
    {
        task_adapter = new OpenSotComAdapter(task, context);
    }
    else if(task->getType() == "Subtask") /* Otherwise, construct supported tasks */
    {
        task_adapter = new OpenSotSubtaskAdapter(task, context);
    }
    else
    {
        auto str = fmt::format("Unable to construct OpenSotTaskAdapter instance for task '{}': "
                               "empty lib name for unrecoginized task type '{}'",
                               task->getName(), task->getType());
        throw std::runtime_error(str);
    }

    Ptr task_shared_ptr(task_adapter);

    return task_shared_ptr;
}

OpenSoT::OptvarHelper OpenSotTaskAdapter::DefaultVars()
{
    OpenSoT::OptvarHelper vars({});
    return vars;
}

OpenSotConstraintAdapter::OpenSotConstraintAdapter(ConstraintDescription::Ptr constr,
                                                   Context::ConstPtr context
                                                   ):
    _ci_constr(constr),
    _model(context->model()),
    _vars({}),
    _ctx(context)
{

}

bool OpenSotConstraintAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    /* Save variables */
    _vars = vars;

    _opensot_constr = constructConstraint();

    if(!_opensot_constr)
    {
        throw std::runtime_error(fmt::format("OpenSotConstraintAdapter: "
                                             "constructConstraint() for constraint '{}' returned null pointer",
                                             _ci_constr->getName()));
    }

    /* Register observer */
    _ci_constr->registerObserver(shared_from_this());

    return true;
}

OpenSoT::OptvarHelper::VariableVector OpenSotConstraintAdapter::getRequiredVariables() const
{
    return {};
}

void OpenSotConstraintAdapter::update(double time, double period)
{

}

ConstraintPtr OpenSotConstraintAdapter::getOpenSotConstraint()
{
    return _opensot_constr;
}

ConstraintDescription::Ptr OpenSotConstraintAdapter::getConstraintDescription() const
{
    return _ci_constr;
}

OpenSotConstraintAdapter::Ptr OpenSotConstraintAdapter::MakeInstance(ConstraintDescription::Ptr constr,
                                                                     Context::ConstPtr context)
{
    OpenSotConstraintAdapter * constr_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(constr->getLibName() != "")
    {
        constr_adapter = CallFunction<OpenSotConstraintAdapter*>(constr->getLibName(),
                                                                 "create_opensot_" + constr->getType() + "_adapter",
                                                                 constr,
                                                                 context,
                                                                 detail::Version CARTESIO_ABI_VERSION);
    }
    else if(constr->getType() == "ConstraintFromTask")
    {
        constr_adapter = new OpenSotConstraintFromTaskAdapter(constr, context);
    }
    else if(constr->getType() == "JointLimits") /* Otherwise, construct supported tasks */
    {
        constr_adapter = new OpenSotJointLimitsAdapter(constr, context);
    }
    else if(constr->getType() == "VelocityLimits")
    {
        constr_adapter = new OpenSotVelocityLimitsAdapter(constr, context);
    }
    else
    {
        auto str = fmt::format("Unable to construct OpenSotConstraintAdapter instance for task '{}': "
                               "empty lib name for unrecoginized constraint type '{}'",
                               constr->getName(), constr->getType());
        throw std::runtime_error(str);
    }

    Ptr constr_shared_ptr(constr_adapter);

    return constr_shared_ptr;
}

OpenSoT::OptvarHelper OpenSotConstraintAdapter::DefaultVars()
{
    OpenSoT::OptvarHelper vars({});
    return vars;
}



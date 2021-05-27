#include "opensot/OpenSotCom.h"

using namespace XBot::Cartesian;

OpenSotComAdapter::OpenSotComAdapter(TaskDescription::Ptr task,
                                     Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_com = std::dynamic_pointer_cast<ComTask>(task);

    if(!_ci_com) throw std::runtime_error("Provided task description "
                                          "does not have expected type 'ComTask'");
}

TaskPtr OpenSotComAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_com = SotUtils::make_shared<ComSoT>(q,
                                              const_cast<ModelInterface&>(*_model));

    return _opensot_com;
}

bool OpenSotComAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotComAdapter] requires default variables definition");
    }

    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;

    /* Register observer */
    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotComAdapter>(shared_from_this());
    _ci_com->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotComAdapter::update(double time, double period)
{
    // note: this will update lambda
    OpenSotTaskAdapter::update(time, period);

    // we implement velocity control by forcing lambda = 0.0
    auto ctrl = _ci_com->getControlMode();

    if(ctrl == ControlType::Velocity)
    {
        _opensot_com->setLambda(0.0);
    }

    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    _ci_com->getPoseReference(Tref, &vref);
    _opensot_com->setReference(Tref.translation(), vref.head<3>()*_ctx->params()->getControlPeriod());
}

bool OpenSotComAdapter::onBaseLinkChanged()
{
    return false;
}

bool OpenSotComAdapter::onControlModeChanged()
{
    return true;
}

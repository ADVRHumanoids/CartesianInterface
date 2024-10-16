#include "opensot/OpenSotCartesian.h"

using namespace XBot::Cartesian;

OpenSotCartesianAdapter::OpenSotCartesianAdapter(TaskDescription::Ptr task,
                                                 Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_cart = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CartesianTask'");

}

TaskPtr OpenSotCartesianAdapter::constructTask()
{
    _opensot_cart = SotUtils::make_shared<CartesianSoT>(_ci_cart->getName(),
                                                     const_cast<ModelInterface&>(*_model),
                                                     _ci_cart->getDistalLink(),
                                                     _ci_cart->getBaseLink());

    return _opensot_cart;
}

bool OpenSotCartesianAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotCartesianAdapter] requires default variables definition");
    }

    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Cartesian task specific parameters */
    _opensot_cart->rotateToLocal(_ci_cart->isSubtaskLocal());

    /* Register observer */
    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotCartesianAdapter>(shared_from_this());
    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotCartesianAdapter::update(double time, double period)
{
    // note: this will update lambda
    OpenSotTaskAdapter::update(time, period);

    // we implement velocity control by forcing lambda = 0.0
    auto ctrl = _ci_cart->getControlMode();

    if(ctrl == ControlType::Velocity)
    {
        _opensot_cart->setLambda(0.0);
    }

    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    _ci_cart->getPoseReference(Tref, &vref);
    if(_ci_cart->isVelocityLocal())
    {
        _opensot_cart->setReference(Tref);
        _opensot_cart->setVelocityLocalReference(vref*_ctx->params()->getControlPeriod());
    }
    else
        _opensot_cart->setReference(Tref, vref*_ctx->params()->getControlPeriod());
}

bool OpenSotCartesianAdapter::onBaseLinkChanged()
{
    return _opensot_cart->setBaseLink(_ci_cart->getBaseLink());
}

bool OpenSotCartesianAdapter::onControlModeChanged()
{
    return true;
}

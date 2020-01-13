#include "OpenSotCartesian.h"

using namespace XBot::Cartesian;

OpenSotCartesianAdapter::OpenSotCartesianAdapter(TaskDescription::Ptr task, XBot::ModelInterface::ConstPtr model):
    OpenSotTaskAdapter(task, model)
{
    _ci_cart = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CartesianTask'");

    Eigen::VectorXd q;
    model->getJointPosition(q);

    _opensot_task = _opensot_cart = boost::make_shared<CartesianSoT>(_ci_cart->getName(),
                                                                     q,
                                                                     const_cast<ModelInterface&>(*model),
                                                                     _ci_cart->getDistalLink(),
                                                                     _ci_cart->getBaseLink());

    _old_lambda = _opensot_task->getLambda();

}

bool OpenSotCartesianAdapter::initialize()
{
    bool ret = OpenSotTaskAdapter::initialize();

    if(!ret) return false;

    auto this_shared_ptr = std::static_pointer_cast<OpenSotCartesianAdapter>(shared_from_this());

    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotCartesianAdapter::update(double time, double period)
{
    /* Update reference */
    Eigen::Affine3d Tref;
    Eigen::Vector6d vref;
    _ci_cart->getPoseReference(Tref, &vref);
    _opensot_cart->setReference(Tref, vref*_ctx.getControlPeriod());
}

bool OpenSotCartesianAdapter::onBaseLinkChanged()
{
    return _opensot_cart->setBaseLink(_ci_cart->getBaseLink());
}

bool OpenSotCartesianAdapter::onControlModeChanged()
{
    auto ctrl = _ci_cart->getControlMode();

    if(ctrl == ControlType::Position)
    {
        _opensot_cart->setLambda(_old_lambda);
    }
    else if(ctrl == ControlType::Velocity)
    {
        _old_lambda = _opensot_cart->getLambda();
        _opensot_cart->setLambda(0.0);
    }

    return true;
}

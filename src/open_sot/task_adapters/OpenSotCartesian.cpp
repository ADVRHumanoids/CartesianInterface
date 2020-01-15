#include "OpenSotCartesian.h"

using namespace XBot::Cartesian;

OpenSotCartesianAdapter::OpenSotCartesianAdapter(TaskDescription::Ptr task,
                                                 XBot::ModelInterface::ConstPtr model):
    OpenSotTaskAdapter(task, model)
{
    _ci_cart = std::dynamic_pointer_cast<CartesianTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'CartesianTask'");

}

TaskPtr OpenSotCartesianAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_cart = boost::make_shared<CartesianSoT>(_ci_cart->getName(),
                                                     q,
                                                     const_cast<ModelInterface&>(*_model),
                                                     _ci_cart->getDistalLink(),
                                                     _ci_cart->getBaseLink());

    return _opensot_cart;
}

bool OpenSotCartesianAdapter::initialize()
{
    bool ret = OpenSotTaskAdapter::initialize();
    if(!ret) return false;


    /* Cartesian task specific parameters */

    _old_lambda = _opensot_cart->getLambda();
    _opensot_cart->setIsBodyJacobian(_ci_cart->isBodyJacobian());

    /* Register observer */

    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotCartesianAdapter>(shared_from_this());
    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotCartesianAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

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

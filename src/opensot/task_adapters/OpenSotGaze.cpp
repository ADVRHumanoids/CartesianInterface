#include "opensot/OpenSotGaze.h"

using namespace XBot::Cartesian;

OpenSotGazeAdapter::OpenSotGazeAdapter(TaskDescription::Ptr task,
                                                 Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_cart = std::dynamic_pointer_cast<GazeTask>(task);

    if(!_ci_cart) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'GazeTask'");

}

TaskPtr OpenSotGazeAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_cart = SotUtils::make_shared<GazeSoT>(_ci_cart->getName(),
                                                     q,
                                                     const_cast<ModelInterface&>(*_model),
                                                     _ci_cart->getBaseLink(),
                                                     _ci_cart->getDistalLink());

    return _opensot_cart;
}

bool OpenSotGazeAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotGazeAdapter] requires default variables definition");
    }

    bool ret = OpenSotTaskAdapter::initialize(vars);
    if(!ret) return false;


    /* Register observer */
    auto this_shared_ptr = std::dynamic_pointer_cast<OpenSotGazeAdapter>(shared_from_this());
    _ci_cart->registerObserver(this_shared_ptr);

    return true;
}

void OpenSotGazeAdapter::update(double time, double period)
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
    _opensot_cart->setGaze(Tref);
}

bool OpenSotGazeAdapter::onBaseLinkChanged()
{
    return _opensot_cart->setBaseLink(_ci_cart->getBaseLink());
}

bool OpenSotGazeAdapter::onControlModeChanged()
{
    return true;
}

#include "opensot/OpenSotVelocityLimits.h"

using namespace XBot::Cartesian;

OpenSotVelocityLimitsAdapter::OpenSotVelocityLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     XBot::ModelInterface::ConstPtr model):
    OpenSotConstraintAdapter (constr, model)
{
    _ci_vlim = std::dynamic_pointer_cast<VelocityLimits>(constr);

    if(!_ci_vlim)
    {
        throw std::runtime_error("Provided constraint description "
                                 "does not have expected type 'VelocityLimits'");
    }

}

ConstraintPtr OpenSotVelocityLimitsAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_vlim = boost::make_shared<VelocityLimitsSoT>(_ci_vlim->getQdotMax(),
                                                 _ctx.getControlPeriod());

    return _opensot_vlim;
}

bool OpenSotVelocityLimitsAdapter::initialize()
{
    OpenSotConstraintAdapter::initialize();

    return true;
}

void OpenSotVelocityLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

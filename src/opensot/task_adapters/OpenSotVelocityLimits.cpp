#include "opensot/OpenSotVelocityLimits.h"

using namespace XBot::Cartesian;

OpenSotVelocityLimitsAdapter::OpenSotVelocityLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter (constr, context)
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
    _opensot_vlim = SotUtils::make_shared<VelocityLimitsSoT>(const_cast<ModelInterface&>(*_model), _ci_vlim->getQdotMax(),
                                                 _ctx->params()->getControlPeriod());

    return _opensot_vlim;
}

bool OpenSotVelocityLimitsAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotVelocityLimitsAdapter] requires default variables definition");
    }

    return OpenSotConstraintAdapter::initialize(vars);
}

void OpenSotVelocityLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

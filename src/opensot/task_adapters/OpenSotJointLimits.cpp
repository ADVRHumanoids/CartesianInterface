#include "opensot/OpenSotJointLimits.h"

using namespace XBot::Cartesian;

OpenSoTJointLimitsInvarianceAdapter::OpenSoTJointLimitsInvarianceAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_jlim = std::dynamic_pointer_cast<JointLimitsInvariance>(constr);

    if(!_ci_jlim)
    {
        throw std::runtime_error("Provided constraint description "
                                 "does not have expected type 'JointLimitsInvariance'");
    }
}

ConstraintPtr OpenSoTJointLimitsInvarianceAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    auto jliSoT =  SotUtils::make_shared<JointLimitsInvarianceSoT>(q,
                                                           _ci_jlim->getQmax(),
                                                           _ci_jlim->getQmin(),
                                                           _ci_jlim->getQddotMax(),
                                                           const_cast<ModelInterface&>(*_model),
                                                           _ctx->params()->getControlPeriod());
    if(!jliSoT->setPStepAheadPredictor(_ci_jlim->getBoundScaling()))
    {
        throw std::runtime_error("Error when setting PStepAheadPredictor value!");
    }
    return jliSoT;
}

bool OpenSoTJointLimitsInvarianceAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotJointLimitsInvarianceAdapter] requires default variables definition");
    }

    return OpenSotConstraintAdapter::initialize(vars);
}

void OpenSoTJointLimitsInvarianceAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}


OpenSotJointLimitsAdapter::OpenSotJointLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     Context::ConstPtr context):
    OpenSotConstraintAdapter (constr, context)
{
    _ci_jlim = std::dynamic_pointer_cast<JointLimits>(constr);

    if(!_ci_jlim)
    {
        throw std::runtime_error("Provided constraint description "
                                 "does not have expected type 'JointLimits'");
    }

}

ConstraintPtr OpenSotJointLimitsAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    return SotUtils::make_shared<JointLimitsSoT>(q,
                                              _ci_jlim->getQmax(),
                                              _ci_jlim->getQmin(),
                                              _ci_jlim->getBoundScaling());
}

bool OpenSotJointLimitsAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotJointLimitsAdapter] requires default variables definition");
    }

    return OpenSotConstraintAdapter::initialize(vars);
}

void OpenSotJointLimitsAdapter::update(double time, double period)
{
    OpenSotConstraintAdapter::update(time, period);
}

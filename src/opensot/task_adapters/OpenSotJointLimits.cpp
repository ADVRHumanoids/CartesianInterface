#include "opensot/OpenSotJointLimits.h"

using namespace XBot::Cartesian;

OpenSotJointLimitsAdapter::OpenSotJointLimitsAdapter(ConstraintDescription::Ptr constr,
                                                     XBot::ModelInterface::ConstPtr model):
    OpenSotConstraintAdapter (constr, model)
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

    return boost::make_shared<JointLimitsSoT>(q,
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

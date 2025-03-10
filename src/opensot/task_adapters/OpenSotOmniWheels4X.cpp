#include "opensot/OpenSotOmniWheels4X.h"

using namespace XBot::Cartesian;

OpenSotOmniWheels4XAdapter::OpenSotOmniWheels4XAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr context):
    OpenSotConstraintAdapter(constr, context)
{
    _ci_omniwheels4x = std::dynamic_pointer_cast<OmniWheels4X>(constr);

    if(!_ci_omniwheels4x) throw std::runtime_error("Provided task description "
                                           "does not have expected type 'OmniWheels4X'");
}

ConstraintPtr OpenSotOmniWheels4XAdapter::constructConstraint()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_omniwheels4x = SotUtils::make_shared<OmniWheels4XSoT>(_ci_omniwheels4x->getl1(),
                                                                   _ci_omniwheels4x->getl2(),
                                                                   _ci_omniwheels4x->getr(),
                                                                   _ci_omniwheels4x->get_joint_wheels_name(),
                                                                   _ci_omniwheels4x->get_base_link(),
                                                                   q, const_cast<ModelInterface&>(*_model));

    return _opensot_omniwheels4x;
}

bool OpenSotOmniWheels4XAdapter::initialize(const OpenSoT::OptvarHelper &vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotOmniWheels4XAdapter] requires default variables definition");
    }

    bool ret = OpenSotConstraintAdapter::initialize(vars);
    if(!ret) return false;

    return true;
}

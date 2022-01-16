#include "opensot/OpenSotSubconstraint.h"

using namespace XBot::Cartesian;

OpenSotSubconstraintAdapter::OpenSotSubconstraintAdapter(ConstraintDescription::Ptr constraint,
                                                         Context::ConstPtr context):
    OpenSotConstraintAdapter(constraint, context)
{
    _ci_subconstraint = std::dynamic_pointer_cast<Subconstraint>(constraint);

    if(!_ci_subconstraint) throw std::runtime_error("Provided constraint description "
                                              "does not have expected type 'Subconstraint'");

    try
    {
        _constraint_adapter = OpenSotConstraintAdapter::MakeInstance(std::dynamic_pointer_cast<ConstraintDescription>(_ci_subconstraint->getTask()),
                                                         context);
    }
    catch(...)
    {
        fprintf(stderr, "Unable to construct constraint from subconstraint \n");
        throw;
    }

}

ConstraintPtr OpenSotSubconstraintAdapter::constructConstraint()
{
    std::list<unsigned int> indices_list(_ci_subconstraint->getSubconstraintIndices().begin(),
                                         _ci_subconstraint->getSubconstraintIndices().end());

    return SotUtils::make_shared<SubconstraintSoT>(_constraint_adapter->getOpenSotConstraint(), indices_list);
}

bool OpenSotSubconstraintAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return _constraint_adapter->initialize(vars) &&
                OpenSotConstraintAdapter::initialize(vars);
}

void OpenSotSubconstraintAdapter::update(double time, double period)
{
    return _constraint_adapter->update(time, period);
}

OpenSoT::OptvarHelper::VariableVector OpenSotSubconstraintAdapter::getRequiredVariables() const
{
    return _constraint_adapter->getRequiredVariables();
}


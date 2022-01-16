#include "problem/Subconstraint.h"

using namespace XBot::Cartesian;


SubconstraintImpl::SubconstraintImpl(TaskDescriptionImpl::Ptr constraint,
                                     std::vector<int> indices,
                                     std::string subconstraint_name):
    ConstraintFromTaskImpl(
        std::make_shared<TaskDescriptionImpl>(
            "Subconstraint", subconstraint_name == "" ? gen_subconstraint_name(constraint, indices) : subconstraint_name, indices.size(), constraint->getContext())),
    _constraint(constraint),
    _indices(indices)
{

}

std::string SubconstraintImpl::gen_subconstraint_name(TaskDescriptionImpl::Ptr constraint, std::vector<int> indices)
{
    std::stringstream ss;
    ss << constraint->getName() << "_subconstraint";

    for(auto i : indices)
    {
        ss << "_" << i;
    }

    return ss.str();
}

TaskDescription::Ptr SubconstraintImpl::getTask()
{
    return _constraint;
}

const std::vector<int>& SubconstraintImpl::getSubconstraintIndices() const
{
    return _indices;
}



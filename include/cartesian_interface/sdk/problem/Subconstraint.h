#ifndef SUBCONSTRAINT_IMPL_H
#define SUBCONSTRAINT_IMPL_H

#include <cartesian_interface/problem/Subconstraint.h>
#include <cartesian_interface/sdk/problem/Constraint.h>

namespace XBot { namespace Cartesian {


class SubconstraintImpl : public virtual Subconstraint,
        public ConstraintFromTaskImpl
{

public:


    SubconstraintImpl(TaskDescriptionImpl::Ptr constraint,
                std::vector<int> indices,
                std::string subconstraint_name = "");

    TaskDescription::Ptr getTask() override;

    const std::vector<int>& getSubconstraintIndices() const override;

private:

    static std::string gen_subconstraint_name(TaskDescriptionImpl::Ptr constraint,
                                    std::vector<int> indices);

    TaskDescriptionImpl::Ptr _constraint;

    std::vector<int> _indices;
};

} }

#endif // SUBTASK_H

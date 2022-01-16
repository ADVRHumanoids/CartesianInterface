#ifndef SUBCONSTRAINT_H
#define SUBCONSTRAINT_H

#include <cartesian_interface/problem/Constraint.h>

namespace XBot { namespace Cartesian {

class Subconstraint : public virtual ConstraintFromTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(Subconstraint)

    virtual TaskDescription::Ptr getTask() = 0;

    virtual const std::vector<int>& getSubconstraintIndices() const = 0;

};

} }

#endif // SUBTASK_H

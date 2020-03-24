#ifndef OPENSOTCONSTRAINTFROMTASK_H
#define OPENSOTCONSTRAINTFROMTASK_H


#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Constraint.h>
#include "OpenSotTask.h"

#include <OpenSoT/constraints/TaskToConstraint.h>


namespace XBot { namespace Cartesian {

class OpenSotConstraintFromTaskAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotConstraintFromTaskAdapter(ConstraintDescription::Ptr constr,
                                     Context::ConstPtr context);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotConstraintFromTaskAdapter() override = default;

protected:

private:

    OpenSotTaskAdapter::Ptr _task_adapter;
};




} }

#endif // OPENSOTCONSTRAINTFROMTASK_H

#ifndef OPENSOTSUBCONSTRAINT_H
#define OPENSOTSUBCONSTRAINT_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Subconstraint.h>

#include "OpenSotTask.h" // contains also OpenSotConstraintAdapter!

#include <OpenSoT/SubConstraint.h>

#include <cartesian_interface/problem/Constraint.h>

using SubconstraintSoT = OpenSoT::SubConstraint;

namespace XBot { namespace Cartesian {

class OpenSotSubconstraintAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotSubconstraintAdapter(ConstraintDescription::Ptr constraint,
                                Context::ConstPtr context);

    OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const override;

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotSubconstraintAdapter() override = default;

private:

    SubconstraintSoT::Ptr _sot_subconstraint;
    Subconstraint::Ptr _ci_subconstraint;
    OpenSotConstraintAdapter::Ptr _constraint_adapter;
};

} }

#endif // OPENSOTSUBCONSTRAINT_H

#ifndef OPENSOTJOINTLIMITS_H
#define OPENSOTJOINTLIMITS_H

#include <memory>

#include <cartesian_interface/problem/Limits.h>
#include "OpenSotTask.h"

#include <OpenSoT/constraints/velocity/JointLimits.h>

using JointLimitsSoT = OpenSoT::constraints::velocity::JointLimits;

namespace XBot { namespace Cartesian {

class OpenSotJointLimitsAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotJointLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr context);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotJointLimitsAdapter() override = default;

protected:

private:

    JointLimitsSoT::Ptr _opensot_jlim;
    JointLimits::Ptr _ci_jlim;
};




} }

#endif // OPENSOTJOINTLIMITS_H

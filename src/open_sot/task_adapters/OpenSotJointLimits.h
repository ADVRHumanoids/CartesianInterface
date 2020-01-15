#ifndef OPENSOTJOINTLIMITS_H
#define OPENSOTJOINTLIMITS_H

#include <boost/make_shared.hpp>

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
                              ModelInterface::ConstPtr model);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize() override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotJointLimitsAdapter() override = default;

protected:

private:

    JointLimitsSoT::Ptr _opensot_jlim;
    JointLimits::Ptr _ci_jlim;
};




} }

#endif // OPENSOTJOINTLIMITS_H
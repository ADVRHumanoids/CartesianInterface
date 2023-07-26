#ifndef OPENSOTJOINTLIMITS_H
#define OPENSOTJOINTLIMITS_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Limits.h>
#include "OpenSotTask.h"

#include <OpenSoT/constraints/velocity/JointLimits.h>
#include <OpenSoT/constraints/velocity/JointLimitsInvariace.h>

using JointLimitsSoT = OpenSoT::constraints::velocity::JointLimits;
using JointLimitsInvarianceSoT = OpenSoT::constraints::velocity::JointLimitsInvariance;

namespace XBot { namespace Cartesian {
class OpenSoTJointLimitsInvarianceAdapter :
        public OpenSotConstraintAdapter
{
public:
    OpenSoTJointLimitsInvarianceAdapter(ConstraintDescription::Ptr constr, Context::ConstPtr context);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSoTJointLimitsInvarianceAdapter() override = default;
private:
    JointLimitsInvarianceSoT::Ptr _opensot_jlim;
    JointLimitsInvariance::Ptr _ci_jlim;
};


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

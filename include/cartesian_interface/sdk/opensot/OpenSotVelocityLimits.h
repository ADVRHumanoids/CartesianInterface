#ifndef OPENSOTVELOCITYLIMITS_H
#define OPENSOTVELOCITYLIMITS_H


#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Limits.h>
#include "OpenSotTask.h"

#include <OpenSoT/constraints/velocity/VelocityLimits.h>

using VelocityLimitsSoT = OpenSoT::constraints::velocity::VelocityLimits;

namespace XBot { namespace Cartesian {

class OpenSotVelocityLimitsAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotVelocityLimitsAdapter(ConstraintDescription::Ptr constr,
                              Context::ConstPtr context);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotVelocityLimitsAdapter() override = default;

protected:

private:

    VelocityLimitsSoT::Ptr _opensot_vlim;
    VelocityLimits::Ptr _ci_vlim;
};




} }

#endif // OPENSOTVELOCITYLIMITS_H

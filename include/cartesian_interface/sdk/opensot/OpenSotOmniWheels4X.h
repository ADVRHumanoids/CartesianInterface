#ifndef OPENSOTOMNIWHEELS4X_H
#define OPENSOTOMNIWHEELS4X_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/sdk/problem/OmniWheels4X.h>
#include "OpenSotTask.h"

#include <OpenSoT/constraints/velocity/OmniWheels4X.h>

using OmniWheels4XSoT = OpenSoT::constraints::velocity::OmniWheels4X;

namespace XBot { namespace Cartesian {

class OpenSotOmniWheels4XAdapter :
        public OpenSotConstraintAdapter
{

public:

    OpenSotOmniWheels4XAdapter(ConstraintDescription::Ptr constr,
                               Context::ConstPtr context);

    virtual ConstraintPtr constructConstraint() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual ~OpenSotOmniWheels4XAdapter() override = default;

protected:

private:

    OmniWheels4XSoT::Ptr _opensot_omniwheels4x;
    OmniWheels4X::Ptr _ci_omniwheels4x;
};




} }

#endif // OPENSOTOMNIWHEELS4X_H

#ifndef OPENSOTCARTESIANADAPTER_H
#define OPENSOTCARTESIANADAPTER_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Cartesian.h>
#include "OpenSotTask.h"

#include <OpenSoT/tasks/velocity/Cartesian.h>

using CartesianSoT = OpenSoT::tasks::velocity::Cartesian;

namespace XBot { namespace Cartesian {

class OpenSotCartesianAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotCartesianAdapter(TaskDescription::Ptr task,
                            Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotCartesianAdapter() override = default;

protected:

    CartesianSoT::Ptr _opensot_cart;

private:

    CartesianTask::Ptr _ci_cart;
};




} }

#endif // OPENSOTTASKADAPTER_H

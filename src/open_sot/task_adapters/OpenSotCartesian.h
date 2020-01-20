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
                            ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize() override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotCartesianAdapter() override = default;

protected:

private:

    CartesianSoT::Ptr _opensot_cart;
    CartesianTask::Ptr _ci_cart;
    double _old_lambda;
};




} }

#endif // OPENSOTTASKADAPTER_H

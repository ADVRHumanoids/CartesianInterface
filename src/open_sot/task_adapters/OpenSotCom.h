#ifndef OPENSOTCOM_H
#define OPENSOTCOM_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Com.h>
#include "OpenSotTask.h"

#include <OpenSoT/tasks/velocity/CoM.h>

using ComSoT = OpenSoT::tasks::velocity::CoM;

namespace XBot { namespace Cartesian {

class OpenSotComAdapter :
        public OpenSotTaskAdapter,
        public CartesianTaskObserver
{

public:

    OpenSotComAdapter(TaskDescription::Ptr task,
                      ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize() override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotComAdapter() override = default;

protected:

private:

    ComSoT::Ptr _opensot_com;
    CartesianTask::Ptr _ci_com;
    double _old_lambda;
};

} }
#endif // OPENSOTCOM_H

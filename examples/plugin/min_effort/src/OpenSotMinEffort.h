#ifndef OPENSOTMinimumEffort_H
#define OPENSOTMinimumEffort_H

#include "MinEffort.h"
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/velocity/MinimumEffort.h>

using MinimumEffortSoT = OpenSoT::tasks::velocity::MinimumEffort;

namespace XBot { namespace Cartesian {


class OpenSotMinimumEffort : public OpenSotTaskAdapter
{

public:

    OpenSotMinimumEffort(TaskDescription::Ptr task,
                           Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotMinimumEffort() override = default;

private:

    MinimumEffortSoT::Ptr _sot_task;
    MinimumEffort::Ptr _ci_task;


};

} }

#endif // OPENSOTMinimumEffort_H

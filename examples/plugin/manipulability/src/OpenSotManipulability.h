#ifndef OPENSOTManipulability_H
#define OPENSOTManipulability_H

#include "Manipulability.h"
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/Task.h>

using ManipulabilitySoT = OpenSoT::tasks::velocity::Manipulability;
using TaskSoT = OpenSoT::Task<Eigen::MatrixXd, Eigen::VectorXd>::TaskPtr;

namespace XBot { namespace Cartesian {


class OpenSotManipulability : public OpenSotTaskAdapter
{

public:

    OpenSotManipulability(TaskDescription::Ptr task,
                           Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotManipulability() override = default;

private:

    ManipulabilitySoT::Ptr _sot_task_manip;
    Manipulability::Ptr _ci_task;
    TaskSoT _sot_task;


};

} }

#endif

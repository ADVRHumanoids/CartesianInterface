#ifndef OPENSOTTASKADAPTER_H
#define OPENSOTTASKADAPTER_H

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/Context.h>

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>


using TaskPtr = OpenSoT::tasks::Aggregated::TaskPtr;
using ConstraintPtr = OpenSoT::constraints::Aggregated::ConstraintPtr;

namespace XBot { namespace Cartesian {

class OpenSotTaskAdapter :
        public std::enable_shared_from_this<OpenSotTaskAdapter>,
        public TaskObserver
{

public:

    OpenSotTaskAdapter(TaskDescription::Ptr task,
                       ModelInterface::ConstPtr model);

    virtual bool initialize();

    virtual void update(double time, double period);

    TaskPtr getOpenSotTask();

    bool onWeightChanged() override;

    bool onActivationStateChanged() override;

    virtual ~OpenSotTaskAdapter() override = default;

protected:

    TaskPtr _opensot_task;
    TaskDescription::Ptr _ci_task;
    Context _ctx;

private:

};


} }

#endif // OPENSOTTASKADAPTER_H

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

    typedef std::shared_ptr<OpenSotTaskAdapter> Ptr;

    virtual TaskPtr constructTask() = 0;


    virtual void update(double time, double period);

    TaskPtr getOpenSotTask();

    virtual bool onWeightChanged() override;

    virtual bool onActivationStateChanged() override;

    virtual ~OpenSotTaskAdapter() override = default;

    static Ptr MakeInstance(TaskDescription::Ptr task,
                            ModelInterface::ConstPtr model);

protected:

    OpenSotTaskAdapter(TaskDescription::Ptr task,
                       ModelInterface::ConstPtr model);

    virtual bool initialize();

    ModelInterface::ConstPtr _model;
    TaskDescription::Ptr _ci_task;
    Context _ctx;

private:


    TaskPtr _opensot_task;
    TaskPtr _sub_task;

};


} }

#endif // OPENSOTTASKADAPTER_H

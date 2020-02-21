#ifndef OPENSOTTASKADAPTER_H
#define OPENSOTTASKADAPTER_H

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/sdk/opensot/Plugin.h>

#include <OpenSoT/tasks/Aggregated.h>
#include <OpenSoT/constraints/Aggregated.h>
#include <OpenSoT/utils/Affine.h>



using TaskPtr = OpenSoT::tasks::Aggregated::TaskPtr;
using ConstraintPtr = OpenSoT::constraints::Aggregated::ConstraintPtr;

namespace XBot { namespace Cartesian {

class OpenSotTaskAdapter :
        public virtual TaskObserver
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
                            ModelInterface::ConstPtr model,
                            const OpenSoT::OptvarHelper& vars = DefaultVars());

protected:

    static OpenSoT::OptvarHelper DefaultVars();

    OpenSotTaskAdapter(TaskDescription::Ptr task,
                       ModelInterface::ConstPtr model,
                       const OpenSoT::OptvarHelper& vars = DefaultVars());

    virtual bool initialize();

    ModelInterface::ConstPtr _model;
    OpenSoT::OptvarHelper _vars;
    TaskDescription::Ptr _ci_task;
    Context _ctx;

private:


    TaskPtr _opensot_task;
    TaskPtr _sub_task;

};

class OpenSotConstraintAdapter :
        public TaskObserver
{

public:

    typedef std::shared_ptr<OpenSotConstraintAdapter> Ptr;

    virtual ConstraintPtr constructConstraint() = 0;

    virtual void update(double time, double period);

    ConstraintPtr getOpenSotConstraint();

    virtual ~OpenSotConstraintAdapter() override = default;

    static Ptr MakeInstance(ConstraintDescription::Ptr constr,
                            ModelInterface::ConstPtr model,
                            const OpenSoT::OptvarHelper& vars = DefaultVars());

protected:

    static OpenSoT::OptvarHelper DefaultVars();

    OpenSotConstraintAdapter(ConstraintDescription::Ptr constr,
                             ModelInterface::ConstPtr model,
                             const OpenSoT::OptvarHelper& vars = DefaultVars());

    virtual bool initialize();

    ModelInterface::ConstPtr _model;
    OpenSoT::OptvarHelper _vars;
    ConstraintDescription::Ptr _ci_constr;
    Context _ctx;

private:

    ConstraintPtr _opensot_constr;

};

} }

#endif // OPENSOTTASKADAPTER_H

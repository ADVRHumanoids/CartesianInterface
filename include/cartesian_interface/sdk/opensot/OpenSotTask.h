#ifndef OPENSOTTASKADAPTER_H
#define OPENSOTTASKADAPTER_H

#include "OpenSotUtils.h"

#include <cartesian_interface/sdk/problem/Task.h>
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


    OpenSotTaskAdapter(TaskDescription::Ptr task,
                       Context::ConstPtr context);

    virtual OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars);

    virtual TaskPtr constructTask() = 0;

    virtual void update(double time, double period);

    virtual void processSolution(const Eigen::VectorXd& solution);

    TaskPtr getOpenSotTask();

    TaskDescriptionImpl::Ptr getTaskDescription() const;

    virtual bool onWeightChanged() override;

    virtual bool onActivationStateChanged() override;

    virtual ~OpenSotTaskAdapter() override = default;

    static Ptr MakeInstance(TaskDescription::Ptr task,
                            Context::ConstPtr context);

protected:

    virtual const Eigen::MatrixXd& getOpenSotWeight() const;

    static OpenSoT::OptvarHelper DefaultVars();
    OpenSoT::OptvarHelper _vars;
    ModelInterface::ConstPtr _model;
    TaskDescriptionImpl::Ptr _ci_task;
    Context::ConstPtr _ctx;

private:

    TaskPtr _opensot_task;
    TaskPtr _sub_task;
    Eigen::VectorXd _task_err;

};

class OpenSotConstraintAdapter :
        public TaskObserver
{

public:

    typedef std::shared_ptr<OpenSotConstraintAdapter> Ptr;

    OpenSotConstraintAdapter(ConstraintDescription::Ptr constr,
                             Context::ConstPtr context);

    virtual OpenSoT::OptvarHelper::VariableVector getRequiredVariables() const;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars);

    virtual ConstraintPtr constructConstraint() = 0;

    virtual void update(double time, double period);

    virtual void processSolution(const Eigen::VectorXd& solution);

    ConstraintPtr getOpenSotConstraint();

    ConstraintDescription::Ptr getConstraintDescription() const;

    virtual ~OpenSotConstraintAdapter() override = default;

    static Ptr MakeInstance(ConstraintDescription::Ptr constr,
                            Context::ConstPtr context);

protected:

    static OpenSoT::OptvarHelper DefaultVars();

    ModelInterface::ConstPtr _model;
    OpenSoT::OptvarHelper _vars;
    ConstraintDescription::Ptr _ci_constr;
    Context::ConstPtr _ctx;

private:

    ConstraintPtr _opensot_constr;
    ConstraintPtr _sub_constr;

};

struct BadVariables : std::runtime_error
{
    using runtime_error::runtime_error;
};

} }

#endif // OPENSOTTASKADAPTER_H

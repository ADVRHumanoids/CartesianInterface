#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <estimation_utils/payload/force_estimation.h>
#include "opensot/OpenSotTask.h"

#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>


namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{

public:

    OpenSotImpl(ProblemDescription ik_problem,
                Context::Ptr context);

    virtual bool update(double time, double period) override;

    virtual ~OpenSotImpl() override;

protected:


private:

    typedef OpenSoT::tasks::Aggregated::TaskPtr TaskPtr;
    typedef OpenSoT::constraints::Aggregated::ConstraintPtr ConstraintPtr;

    void make_task_adapter(TaskDescription::Ptr);
    void make_constraint_adapter(ConstraintDescription::Ptr constr_desc);
    TaskPtr aggregated_from_stack(AggregatedTask stack);

    Eigen::VectorXd _qref;
    Eigen::VectorXd _x, _q, _dq, _ddq, _tau;
    Eigen::MatrixXd _J;

    std::vector<OpenSotTaskAdapter::Ptr> _task_adapters;
    std::vector<OpenSotConstraintAdapter::Ptr> _constr_adapters;
    OpenSoT::OptvarHelper _vars;
    std::map<std::string, OpenSoT::AffineHelper> _vars_map;

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    Eigen::MatrixXd _js_inertia_inv;
    bool _force_space_references;

    XBot::MatLogger2::Ptr _logger;

};


} }

#endif

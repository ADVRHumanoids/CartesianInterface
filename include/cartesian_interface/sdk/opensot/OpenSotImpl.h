#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>


namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{
    
public:

    /* Typedefs for shared pointers */
    CARTESIO_DECLARE_SMART_PTR(OpenSotImpl)
    
    OpenSotImpl(ProblemDescription ik_problem,
                Context::Ptr context);

    virtual bool update(double time, double period) override;
    
    virtual ~OpenSotImpl() override;

    OpenSoT::tasks::Aggregated::TaskPtr getOpenSotTask(const std::string& task_name);
    OpenSoT::constraints::Aggregated::ConstraintPtr getOpenSotConstraint(const std::string& constr_name);
    
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
    std::map<std::string, OpenSoT::tasks::Aggregated::TaskPtr> _open_sot_tasks;
    std::map<std::string, OpenSoT::constraints::Aggregated::ConstraintPtr> _open_sot_constraints;

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    Eigen::MatrixXd _js_inertia_inv;
    bool _force_space_references;
    
    XBot::MatLogger2::Ptr _logger;

};


} }

#endif

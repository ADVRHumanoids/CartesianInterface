#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include "opensot/OpenSotTask.h"

#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>


namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{
    
public:
    
    OpenSotImpl(ModelInterface::Ptr model,
                ProblemDescription ik_problem);

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
    Eigen::VectorXd _x, _q, _dq, _ddq;
    
    std::vector<OpenSotTaskAdapter::Ptr> _task_adapters;
    std::vector<OpenSotConstraintAdapter::Ptr> _constr_adapters;
    OpenSoT::OptvarHelper _vars;
    std::map<std::string, OpenSoT::AffineHelper> _vars_map;

    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;

};


} }

#endif

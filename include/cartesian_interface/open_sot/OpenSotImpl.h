#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>

namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{
    
public:
    
    OpenSotImpl(ModelInterface::Ptr model, ProblemDescription ik_problem);

    virtual bool update(double time, double period);
    virtual bool solve();

    virtual ~OpenSotImpl();
    
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);
    
protected:
    
    
private:
    
    typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
    typedef OpenSoT::tasks::velocity::CoM CoMTask;
    
    void set_adaptive_lambda(CartesianTask::Ptr cartesian_task);
    
    OpenSoT::tasks::Aggregated::Ptr aggregated_from_stack(AggregatedTask stack);
    OpenSoT::constraints::Aggregated::ConstraintPtr constraint_from_description(ConstraintDescription::Ptr constr_desc);
    
    Eigen::VectorXd _q, _dq, _ddq;
    
    std::vector<CartesianTask::Ptr> _cartesian_tasks;
    CoMTask::Ptr _com_task;
    OpenSoT::tasks::velocity::Postural::Ptr _postural_task;
    
    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;
    
};


} }

#endif
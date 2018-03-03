#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <CartesianInterface/CartesianInterfaceImpl.h>
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

    virtual ~OpenSotImpl();
    
    
protected:
    
    
private:
    
    OpenSoT::tasks::Aggregated::Ptr aggregated_from_stack(AggregatedTask stack);
    OpenSoT::constraints::Aggregated::ConstraintPtr constraint_from_description(ConstraintDescription::Ptr constr_desc);
    
    Eigen::VectorXd _q, _dq, _ddq;
    
    std::vector<OpenSoT::tasks::velocity::Cartesian::Ptr> _cartesian_tasks;
    
    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;
    
};


} }

#endif
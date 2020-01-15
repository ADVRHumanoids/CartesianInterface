#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include "open_sot/task_adapters/OpenSotTask.h"

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
    
    TaskPtr construct_task(TaskDescription::Ptr);
    TaskPtr aggregated_from_stack(AggregatedTask stack);
    ConstraintPtr constraint_from_description(ConstraintDescription::Ptr constr_desc);
    
    Eigen::VectorXd _qref;
    Eigen::VectorXd _q, _dq, _ddq;
    
    std::vector<OpenSotTaskAdapter::Ptr> _task_adapters;

    Utils::ForceEstimation::Ptr _force_estimation;
    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;

};


} }

#endif

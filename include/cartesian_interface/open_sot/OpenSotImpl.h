#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Gaze.h>
#include <OpenSoT/tasks/velocity/AngularMomentum.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>

namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{
    
public:
    
    OpenSotImpl(ModelInterface::Ptr model, ProblemDescription ik_problem);

    virtual bool update(double time, double period);

    virtual ~OpenSotImpl();
    
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);
    virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type);
    
protected:
    
    
private:
    
    typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
    typedef OpenSoT::tasks::velocity::Postural PosturalTask;
    typedef OpenSoT::tasks::velocity::CoM CoMTask;
    typedef OpenSoT::tasks::velocity::Gaze GazeTask;
    typedef OpenSoT::tasks::velocity::AngularMomentum AngularMomentumTask;
    typedef OpenSoT::tasks::Aggregated::TaskPtr TaskPtr;
    typedef OpenSoT::constraints::Aggregated::ConstraintPtr ConstraintPtr;
    
    void set_adaptive_lambda(CartesianTask::Ptr cartesian_task);
    
    TaskPtr construct_task(TaskDescription::Ptr);
    TaskPtr aggregated_from_stack(AggregatedTask stack);
    ConstraintPtr constraint_from_description(ConstraintDescription::Ptr constr_desc);
    
    Eigen::VectorXd _qref;
    Eigen::VectorXd _q, _dq, _ddq;
    
    std::vector<CartesianTask::Ptr> _cartesian_tasks;
    std::vector<PosturalTask::Ptr> _postural_tasks;
    std::vector<bool> _use_inertia_matrix;
    std::map<std::string, double> _lambda_map;
    CoMTask::Ptr _com_task;
    GazeTask::Ptr _gaze_task;
    AngularMomentumTask::Ptr _angular_momentum_task;
    
    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;

    /**
     * @brief _B general coordinate inertia matrix
     */
    Eigen::MatrixXd _B;
    
};


} }

#endif

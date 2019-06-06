#ifndef __XBOT_CARTESIAN_OPENSOT_ACC_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_ACC_IMPL_H__


#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/ros/RosEnabled.h>
#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <cartesian_interface/open_sot/TaskInterface.h>

#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Gaze.h>
#include <OpenSoT/tasks/velocity/AngularMomentum.h>
#include <OpenSoT/Solver.h>
#include <OpenSoT/utils/AutoStack.h>

#include <std_msgs/Float32.h>

//ADDED
#include <OpenSoT/utils/InverseDynamics.h>
#include <OpenSoT/tasks/acceleration/Cartesian.h>
#include <OpenSoT/tasks/acceleration/Postural.h>
#include <OpenSoT/tasks/acceleration/Contact.h>


namespace XBot { namespace Cartesian {

class OpenSotAccImpl : public CartesianInterfaceImpl, 
                       public RosEnabled
{
    
public:
    
    OpenSotAccImpl(ModelInterface::Ptr model, ProblemDescription ik_problem);

    virtual bool update(double time, double period);
    
    virtual bool initRos(ros::NodeHandle nh);
    virtual void updateRos();

    virtual ~OpenSotAccImpl();
    
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);
    virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type);
    
protected:
    
    
private:
    
    typedef OpenSoT::tasks::velocity::Cartesian CartesianTask;
    typedef OpenSoT::tasks::velocity::CartesianAdmittance CartesianAdmittanceTask;
    typedef OpenSoT::tasks::velocity::Postural PosturalTask;
    typedef OpenSoT::tasks::velocity::CoM CoMTask;
    typedef OpenSoT::tasks::velocity::Gaze GazeTask;
    typedef OpenSoT::tasks::velocity::AngularMomentum AngularMomentumTask;
    typedef OpenSoT::tasks::Aggregated::TaskPtr TaskPtr;
    typedef OpenSoT::constraints::Aggregated::ConstraintPtr ConstraintPtr;
    
    typedef OpenSoT::tasks::acceleration::Cartesian CartesianAccTask;
    typedef OpenSoT::tasks::acceleration::Postural PosturalAccTask;
    
    void set_adaptive_lambda(CartesianTask::Ptr cartesian_task);
    bool get_control_dt(double& dt);
    
    TaskPtr construct_task(TaskDescription::Ptr);
    TaskPtr aggregated_from_stack(AggregatedTask stack);
    ConstraintPtr constraint_from_description(ConstraintDescription::Ptr constr_desc);
    
    void lambda_callback(const std_msgs::Float32ConstPtr& msg, const std::string& ee_name);
    
    Eigen::VectorXd _qref;
    Eigen::VectorXd _q, _dq, _ddq, _x;
    
    std::vector<SoT::TaskInterface::Ptr> _task_ifc;
    std::vector<CartesianAccTask::Ptr> _cartesian_tasks, _interaction_tasks;
    std::vector<CartesianAdmittanceTask::Ptr> _admittance_tasks;
    std::vector<PosturalAccTask::Ptr> _postural_tasks;
    std::vector<bool> _use_inertia_matrix;
    CoMTask::Ptr _com_task;
    GazeTask::Ptr _gaze_task;
    AngularMomentumTask::Ptr _angular_momentum_task;
    bool _minimize_rate_of_change;
    
    Utils::ForceEstimation::Ptr _force_estimation;
    OpenSoT::Solver<Eigen::MatrixXd, Eigen::VectorXd>::SolverPtr _solver;
    OpenSoT::AutoStack::Ptr _autostack;
    
    XBot::MatLogger::Ptr _logger;

    /**
     * @brief _B general coordinate inertia matrix
     */
    Eigen::MatrixXd _B;
    
    /* TBD must become thread safe */
    std::map<std::string, ros::Subscriber> _lambda_sub_map;
    std::map<std::string, double> _lambda_map;
    bool _update_lambda;
    
    // ADDED
    /**
     * @brief _id inverse dynamics computation & variable helper
     */
    OpenSoT::utils::InverseDynamics::Ptr _id;
    
    /**
     * @brief _qddot optimization variable helper
     */
    OpenSoT::AffineHelper _qddot;
    
    std::vector<std::string> _links_in_contact;
    
    Eigen::VectorXd _tau;
    
    double _l1, _l2, _l1_old, _l2_old;
};


} }

#endif

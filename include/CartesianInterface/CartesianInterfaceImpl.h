#ifndef __XBOT_CARTESIAN_IMPL_H__
#define __XBOT_CARTESIAN_IMPL_H__


#include <CartesianInterface/CartesianInterface.h>
#include <CartesianInterface/trajectory/Trajectory.h>
#include <CartesianInterface/ProblemDescription.h>

namespace XBot { namespace Cartesian {

class CartesianInterfaceImpl : public CartesianInterface 
{
    
public:
    
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                           std::vector<std::pair<std::string, std::string>> tasks);
    
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                           ProblemDescription ik_problem);
    
    virtual bool getPoseReference(const std::string& end_effector, 
                                Eigen::Affine3d& w_T_ref, 
                                Eigen::Vector6d& w_vel_ref, 
                                Eigen::Vector6d& w_acc_ref) const;

    virtual bool getPoseTarget(const std::string& end_effector, 
                            Eigen::Affine3d& w_T_ref) const;
                            
    virtual bool setComPositionReference(const Eigen::Vector3d& w_com_ref, 
                                         const Eigen::Vector3d& w_vel_ref = Eigen::Vector3d::Zero(), 
                                         const Eigen::Vector3d& w_acc_ref = Eigen::Vector3d::Zero(), 
                                         ControlType control_type = ControlType::Position);

    virtual bool setPoseReference(const std::string& end_effector, 
                                  const Eigen::Affine3d& w_T_ref, 
                                  const Eigen::Vector6d& w_vel_ref = Eigen::Vector6d::Zero(), 
                                  const Eigen::Vector6d& w_acc_ref = Eigen::Vector6d::Zero(), 
                                  ControlType control_type = ControlType::Position);

    virtual bool setPositionReference(const std::string& end_effector, 
                                    const Eigen::Vector3d& w_pos_ref, 
                                    const Eigen::Vector3d& w_vel_ref = Eigen::Vector3d::Zero(), 
                                    const Eigen::Vector3d& w_acc_ref = Eigen::Vector3d::Zero(), 
                                    ControlType control_type = ControlType::Position);

    virtual bool setTargetComPosition(const Eigen::Vector3d& w_com_ref, double time = 0, 
                                    const Eigen::Vector3d& max_velocity = Eigen::Vector3d::Zero());

    virtual bool setTargetOrientation(const std::string& end_effector, 
                                    const std::string& base_frame, 
                                    const Eigen::Matrix3d& base_R_ref, 
                                    double time = 0, 
                                    const Eigen::Vector3d& max_velocity = Eigen::Vector3d::Zero());

    virtual bool setTargetOrientation(const std::string& end_effector, 
                                    const Eigen::Vector3d& w_pos_ref, 
                                    double time = 0, 
                                    const Eigen::Vector3d& max_velocity = Eigen::Vector3d::Zero());

    virtual bool setTargetPose(const std::string& end_effector, 
                            const Eigen::Affine3d& w_T_ref, 
                            double time = 0, 
                            const Eigen::Vector6d& max_velocity = Eigen::Vector6d::Zero());

    virtual bool setTargetPosition(const std::string& end_effector, 
                                const Eigen::Vector3d& w_pos_ref, 
                                double time = 0, 
                                const Eigen::Vector3d& max_velocity = Eigen::Vector3d::Zero());

    ~CartesianInterfaceImpl();

    virtual bool update(double time, double period);

    virtual bool abort(const std::string& end_effector);
    
    virtual bool reset();


    
protected:
    
    struct Task
    {
        std::string base_frame;
        std::string distal_frame;
        
        Eigen::Affine3d T;
        Eigen::Vector6d vel;
        Eigen::Vector6d acc;
        
        ControlType control_type;
        State state;
        
        Trajectory::Ptr trajectory;
        
        typedef std::shared_ptr<Task> Ptr;
        Task();
    };
    
    double get_current_time() const;
    
    const std::map<std::string, Task::Ptr>& get_tasks() const;
    
    XBot::ModelInterface::Ptr _model;
    
    
private:
    
    void __construct_from_vectors();
    void log_tasks();
    
    std::vector<std::pair<std::string, std::string>> _tasks_vector;
    
    Task::Ptr get_task(const std::string& ee_name) const;
    
    std::map<std::string, Task::Ptr> _task_map;
    
    double _current_time;
    
    XBot::MatLogger::Ptr _logger;
    
};


} }


#endif






















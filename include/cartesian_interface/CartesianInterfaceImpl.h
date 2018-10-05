/*
 * Copyright (C) 2018 IIT-ADVR
 * Author: Arturo Laurenzi
 * email:  arturo.laurenzi@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __XBOT_CARTESIAN_IMPL_H__
#define __XBOT_CARTESIAN_IMPL_H__


#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/trajectory/Trajectory.h>
#include <cartesian_interface/problem/ProblemDescription.h>
#include <XBotInterface/SoLib.h>
#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian {

class CartesianInterfaceImpl : public CartesianInterface 
{
    
public:
    
    /* Typedefs for shared pointers */
    typedef std::shared_ptr<CartesianInterfaceImpl> Ptr;
    typedef std::shared_ptr<const CartesianInterfaceImpl> ConstPtr;
    
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                           std::vector<std::pair<std::string, std::string>> tasks);
    
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                           ProblemDescription ik_problem);
    
    void syncFrom(CartesianInterfaceImpl::ConstPtr other);
    
    ModelInterface::Ptr getModel() const;
    
    virtual const std::vector<std::string>& getTaskList() const;
    virtual const std::string& getBaseLink(const std::string& ee_name) const;
    
    virtual ControlType getControlMode(const std::string& ee_name) const;
    virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type);
    virtual void getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const;
    virtual void getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const;
    virtual void setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang);
    virtual void setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang);
    virtual void enableOtg(double expected_dt);
    
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);
    
    virtual bool getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const;
                          
    bool getPoseReferenceRaw(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const;

    virtual bool getPoseTarget(const std::string& end_effector, 
                            Eigen::Affine3d& base_T_ref) const;
                            
    virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, 
                                         Eigen::Vector3d* base_vel_ref = nullptr, 
                                         Eigen::Vector3d* base_acc_ref = nullptr) const;
                                         
    virtual bool getReferencePosture(Eigen::VectorXd& qref) const;                                     
    virtual bool getReferencePosture(JointNameMap& qref) const;

    virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const;
                            
    virtual bool getCurrentPose(const std::string& end_effector, Eigen::Affine3d& base_T_ee) const;
                            
    virtual State getTaskState(const std::string& end_effector) const;
                            
    virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref, 
                                         const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(), 
                                         const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero());

    virtual bool setPoseReference(const std::string& end_effector, 
                                  const Eigen::Affine3d& base_T_ref, 
                                  const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(), 
                                  const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero());
    
    bool setPoseReferenceRaw(const std::string& end_effector, 
                                  const Eigen::Affine3d& base_T_ref, 
                                  const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(), 
                                  const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero());

    virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time = 0);

    virtual bool setTargetPose(const std::string& end_effector, 
                            const Eigen::Affine3d& base_T_ref, 
                            double time = 0);
    
    virtual bool setWayPoints(const std::string& end_effector, 
                       const Trajectory::WayPointVector& way_points
                      );

    virtual bool setTargetPosition(const std::string& end_effector, 
                                const Eigen::Vector3d& base_pos_ref, 
                                double time = 0);
    
    virtual bool setReferencePosture(const JointNameMap& qref);

    ~CartesianInterfaceImpl();

    virtual bool update(double time, double period);

    virtual bool abort(const std::string& end_effector);
    
    virtual bool reset(double time);
    virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);


    
protected:
    
    class Task
    {
        
    public:
        
        typedef std::shared_ptr<Task> Ptr;
        typedef std::shared_ptr<const Task> ConstPtr;
        typedef Reflexxes::Utils::TrajectoryGenerator OtgType;
        
        Task();
        Task(const std::string& base, const std::string& distal);
        
        void update(double time, double period);
        
        const std::string& get_name() const;
        const std::string& get_base() const;
        const std::string& get_distal() const;
        State get_state() const;
        ControlType get_ctrl() const;
        const Eigen::Affine3d& get_pose() const;
        const Eigen::Affine3d get_pose_otg() const;
        const Eigen::Vector6d& get_velocity() const;
        const Eigen::Vector6d& get_acceleration() const;
        bool get_pose_target(Eigen::Affine3d& pose_target) const;
        bool is_new_data_available() const;
        
        
        
        void set_ctrl(ControlType ctrl, ModelInterface::ConstPtr model);
        bool set_reference(const Eigen::Affine3d& pose, 
                           const Eigen::Vector6d& vel, 
                           const Eigen::Vector6d& acc);
        bool set_reference_raw(const Eigen::Affine3d& pose, 
                           const Eigen::Vector6d& vel, 
                           const Eigen::Vector6d& acc);
        bool set_waypoints(double current_time, const Trajectory::WayPointVector& wp);
        bool set_target_pose(double current_time, double target_time, const Eigen::Affine3d& pose);
        void reset(ModelInterface::ConstPtr model);
        void sync_from(const Task& other);
        void set_otg_dt(double expected_dt);
        void get_otg_vel_limits(double& linear, double& angular) const;
        void get_otg_acc_limits(double& linear, double& angular) const;
        void set_otg_vel_limits(double linear, double angular);
        void set_otg_acc_limits(double linear, double angular);
        
        void abort();
        bool change_base_link(const std::string& new_base_link, ModelInterface::ConstPtr model);
        
        void reset_otg();
        
    private:
        
        typedef Eigen::Matrix<double, 7, 1> EigenVector7d;
        
        bool check_reach() const;
        
        void apply_otg();
        
        std::string base_frame;
        std::string distal_frame;
        
        Eigen::Affine3d T;
        Eigen::Vector6d vel;
        Eigen::Vector6d acc;
        
        EigenVector7d __otg_des, __otg_ref, __otg_vref;
        EigenVector7d __otg_maxvel, __otg_maxacc;
        
        ControlType control_type;
        State state;
        double vref_time_to_live;
        
        bool new_data_available;
        
        Trajectory::Ptr trajectory;
        Reflexxes::Utils::TrajectoryGenerator::Ptr otg;
        
        
    };
    
    double get_current_time() const;
    
    bool has_config() const;
    const YAML::Node& get_config() const;
    
    XBot::ModelInterface::Ptr _model;
    
    
private:
    
    void add_task(TaskDescription::Ptr task);
    
    void __construct_from_vectors();
    void log_tasks();
    void init_log_tasks();
    
    std::vector<std::pair<std::string, std::string>> _tasks_vector;
    std::vector<std::string> _ee_list;
    
    Task::Ptr get_task(const std::string& ee_name) const;
    bool postural_task_defined() const;
    
    std::map<std::string, Task::Ptr> _task_map;
    Task::Ptr _com_task;
    Eigen::VectorXd _q_ref;
    
    double _current_time;
    
    XBot::MatLogger::Ptr _logger;
    
    YAML::Node _solver_options;
    
};


} }


#endif





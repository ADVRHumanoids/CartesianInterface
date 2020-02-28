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
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>

namespace XBot { namespace Cartesian {

class CartesianInterfaceImpl : public CartesianInterface 
{
    
public:
    
    /* Typedefs for shared pointers */
    CARTESIO_DECLARE_SMART_PTR(CartesianInterfaceImpl)
    
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                           ProblemDescription ik_problem);

    static Ptr MakeInstance(std::string solver_name,
                            XBot::ModelInterface::Ptr model,
                            ProblemDescription ik_problem);
    
    void syncFrom(CartesianInterfaceImpl::ConstPtr other);
    
    ModelInterface::Ptr getModel() const;
    const ProblemDescription& getIkProblem() const;
    
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

    virtual ActivationState getActivationState(const std::string& ee_name) const;

    virtual bool setActivationState(const std::string& ee_name, ActivationState activ_state);

    
    virtual bool getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const;
                          
    virtual bool getDesiredInteraction(const std::string& end_effector, 
                          Eigen::Vector6d& force, 
                          Eigen::Matrix6d& stiffness,
                          Eigen::Matrix6d& damping) const;
                          
    virtual bool getPoseReferenceRaw(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const;

    virtual bool getPoseTarget(const std::string& end_effector, 
                            Eigen::Affine3d& base_T_ref) const;

    virtual int getCurrentSegmentId(const std::string& end_effector) const;
                            
    virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, 
                                         Eigen::Vector3d* base_vel_ref = nullptr, 
                                         Eigen::Vector3d* base_acc_ref = nullptr) const;
                                         
    virtual bool getReferencePosture(Eigen::VectorXd& qref) const;                                     
    virtual bool getReferencePosture(JointNameMap& qref) const;

    virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const;
                            
    virtual bool getCurrentPose(const std::string& end_effector, Eigen::Affine3d& base_T_ee) const;
                            
    virtual State getTaskState(const std::string& end_effector) const;
                            
    virtual bool setPoseReference(const std::string& end_effector, 
                          const Eigen::Affine3d& base_T_ref);
                          
    virtual bool setVelocityReference(const std::string& end_effector, 
                          const Eigen::Vector6d& base_vel_ref);
    
    virtual bool setForceReference(const std::string& end_effector,
                                   const Eigen::Vector6d& force);
                                   
    virtual bool setDesiredStiffness(const std::string& end_effector,
                                   const Eigen::Matrix6d& k);
                                   
    virtual bool setDesiredDamping(const std::string& end_effector,
                                   const Eigen::Matrix6d& d);
                          
    virtual bool setPoseReferenceRaw(const std::string& end_effector, 
                             const Eigen::Affine3d& base_T_ref);
    
    virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref);
                                 
    virtual bool setComVelocityReference(const Eigen::Vector3d& base_vel_ref);

    virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time = 0);

    virtual bool setTargetPose(const std::string& end_effector, 
                               const Eigen::Affine3d& base_T_ref, 
                               double time = 0);
    
    virtual bool setWayPoints(const std::string& end_effector, 
                              const Trajectory::WayPointVector& way_points
                            );

    virtual bool setReferencePosture(const JointNameMap& qref);


    virtual bool update(double time, double period);

    virtual bool abort(const std::string& end_effector);
    
    virtual bool reset(double time);
    virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);

    virtual TaskDescription::Ptr getTask(const std::string& task_name);

    ~CartesianInterfaceImpl();
    
protected:
    
    double get_current_time() const;
    
    bool has_config() const;
    const YAML::Node& get_config() const;
    
    XBot::ModelInterface::Ptr _model;
    ProblemDescription _ik_problem;
    
private:
    

    void add_task(TaskDescription::Ptr task);
    void log_tasks();
    void init_log_tasks();
    
   
    std::vector<std::string> _task_list;
    
    TaskDescription::Ptr get_task(const std::string& name) const;
    CartesianTask::Ptr get_cart_task(const std::string& name) const;
    ComTask::Ptr get_com_task() const;
    
    std::map<std::string, TaskDescription::Ptr> _task_map;
    std::map<std::string, CartesianTask::Ptr> _cart_task_map;
    std::map<std::string, ComTask::Ptr> _com_task_map;
    std::map<std::string, PosturalTask::Ptr> _postural_task_map;
    
    double _current_time;
    
    XBot::MatLogger::Ptr _logger;
    
    YAML::Node _solver_options;
    
    bool postural_task_defined() const;
};


} }


#endif





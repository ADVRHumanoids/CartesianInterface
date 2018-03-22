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
#include <cartesian_interface/ProblemDescription.h>
#include <XBotInterface/SoLib.h>

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
    
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);
    
    virtual bool getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const;

    virtual bool getPoseTarget(const std::string& end_effector, 
                            Eigen::Affine3d& base_T_ref) const;
                            
    virtual bool getCurrentPose(const std::string& end_effector, Eigen::Affine3d& base_T_ee) const;
                            
    virtual State getTaskState(const std::string& end_effector) const;
                            
    virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref, 
                                         const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(), 
                                         const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero());

    virtual bool setPoseReference(const std::string& end_effector, 
                                  const Eigen::Affine3d& base_T_ref, 
                                  const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(), 
                                  const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero());

    virtual bool setPositionReference(const std::string& end_effector, 
                                    const Eigen::Vector3d& base_pos_ref, 
                                    const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(), 
                                    const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero());

    virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time = 0);

    virtual bool setTargetOrientation(const std::string& end_effector, 
                                    const Eigen::Matrix3d& base_R_ref, 
                                    double time = 0);

    virtual bool setTargetPose(const std::string& end_effector, 
                            const Eigen::Affine3d& base_T_ref, 
                            double time = 0);

    virtual bool setTargetPosition(const std::string& end_effector, 
                                const Eigen::Vector3d& base_pos_ref, 
                                double time = 0);

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
        typedef std::shared_ptr<const Task> ConstPtr;
        Task();
    };
    
    double get_current_time() const;
    
    const std::map<std::string, Task::Ptr>& get_tasks() const;
    
    XBot::ModelInterface::Ptr _model;
    
    
private:
    
    void __construct_from_vectors();
    void log_tasks();
    
    std::vector<std::pair<std::string, std::string>> _tasks_vector;
    std::vector<std::string> _ee_list;
    
    Task::Ptr get_task(const std::string& ee_name) const;
    
    std::map<std::string, Task::Ptr> _task_map;
    
    double _current_time;
    
    XBot::MatLogger::Ptr _logger;
    
};


} }


#endif























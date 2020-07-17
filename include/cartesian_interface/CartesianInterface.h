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

#ifndef __XBOT_CARTESIAN_INTERFACE_H__
#define __XBOT_CARTESIAN_INTERFACE_H__

#include <XBotInterface/ModelInterface.h>

#include <cartesian_interface/trajectory/Trajectory.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/Enum.h>

namespace XBot { namespace Cartesian {

/**
* @brief The CartesianInterface class provides interfaces
* to perform cartesian control of floating base robots in
* a generic fashion.
*
* The recommended way to interact with the defined tasks
* and constraint is to use their own specific APIs, as
* obtained via the getTask() method.
*/
class CartesianInterface
{

public:

    /* Typedefs for shared pointers */
    typedef std::shared_ptr<CartesianInterface> Ptr;
    typedef std::shared_ptr<const CartesianInterface> ConstPtr;
    
    /* CartesianInterface cannot be copied */
    CartesianInterface() = default;
    CartesianInterface(const CartesianInterface& other) = delete;
    CartesianInterface(const CartesianInterface&& other) = delete;
    CartesianInterface& operator=(const CartesianInterface& rhs) = delete;
    CartesianInterface& operator=(const CartesianInterface&& rhs) = delete;
    
    
    /**
     * @brief Update function. It must be called periodically inside the user
     * control loop
     * 
     * @param time current time [s]
     * @param period period between last call and current one [s]
     * @return true if no errors occurred
     */
    virtual bool update(double time, double period) = 0;
    
    /**
     * @brief Reset all cartesian references to current model pose.
     * It also resets time.
     * 
     * @param time Current time
     */
    virtual bool reset(double time) = 0;
    
    /**
     * @brief Reset the world frame and call reset().
     *
     * @param w_T_new_world is the new world pose w.r.t. old world
     */
    virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world) = 0;
    
    /**
     * @brief getTaskList returns the list of all defined tasks and
     * constraints; returned elements may be used as arguments to getTask()
     */
    virtual const std::vector<std::string>& getTaskList() const = 0;

    /**
     * @brief getTask returns a shared pointer to the task or constraint
     * with given name, if it exists. It returns a null shared pointer
     * otherwise. From CartesI/O v2.0 on, the recommended way to interact
     * with tasks and constraints is to use the returned task handle,
     * casted to the appropriate type (e.g. 'CartesianTask') via
     * std::dynamic_pointer_task<>()
     * @param task_name
     * @return a shared pointer to the task or constraint
     */
    virtual TaskDescription::Ptr getTask(const std::string& task_name) = 0;
    
    /* All functions below should be deprecated, please call the equivalent
     * method on a TaskDescription::Ptr!
     */

    virtual const std::string& getBaseLink(const std::string& ee_name) const = 0;

    virtual ActivationState getActivationState(const std::string& ee_name) const = 0;

    virtual bool setActivationState(const std::string& ee_name,
                                    ActivationState activ_state) = 0;

    virtual bool setControlMode(const std::string& ee_name,
                                ControlType ctrl_type) = 0;
    
    virtual ControlType getControlMode(const std::string& ee_name) const = 0;
    
    virtual State getTaskState(const std::string& end_effector) const = 0;
    
    virtual bool setBaseLink(const std::string& ee_name,
                             const std::string& new_base_link) = 0;
    
    /* Point-to-point control */
    
    virtual bool setTargetPose(const std::string& end_effector, 
                       const Eigen::Affine3d& base_T_ref, 
                       double time = 0
                               ) = 0;

    virtual bool setWayPoints(const std::string& end_effector, 
                       const Trajectory::WayPointVector& way_points
                              ) = 0;
    
    virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, 
                              double time = 0) = 0;
    
    virtual bool abort(const std::string& end_effector) = 0;
    
    virtual void getVelocityLimits(const std::string& ee_name,
                                   double& max_vel_lin,
                                   double& max_vel_ang) const = 0;

    virtual void getAccelerationLimits(const std::string& ee_name,
                                       double& max_acc_lin,
                                       double& max_acc_ang) const = 0;

    virtual void setVelocityLimits(const std::string& ee_name,
                                   double max_vel_lin,
                                   double max_vel_ang) = 0;

    virtual void setAccelerationLimits(const std::string& ee_name,
                                       double max_acc_lin,
                                       double max_acc_ang) = 0;
    
    
    /* Online control */
    
    virtual bool setForceReference(const std::string& end_effector,
                                   const Eigen::Vector6d& force) = 0;

    virtual bool setDesiredStiffness(const std::string& end_effector,
                                     const Eigen::Matrix6d& k) = 0;

    virtual bool setDesiredDamping(const std::string& end_effector,
                                   const Eigen::Matrix6d& d) = 0;

    virtual bool setPoseReference(const std::string& end_effector, 
                          const Eigen::Affine3d& base_T_ref) = 0;

    virtual bool setVelocityReference(const std::string& end_effector, 
                          const Eigen::Vector6d& base_vel_ref) = 0;

    virtual bool setPoseReferenceRaw(const std::string& end_effector, 
                             const Eigen::Affine3d& base_T_ref) = 0;
    
    virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref) = 0;

    virtual bool setComVelocityReference(const Eigen::Vector3d& base_vel_ref) = 0;

    virtual bool setReferencePosture(const JointNameMap& qref) = 0;
    
    /* Monitoring */
    
    virtual bool getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    virtual bool getPoseReferenceRaw(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const = 0;

    virtual bool getDesiredInteraction(const std::string& end_effector, 
                          Eigen::Vector6d& force, 
                          Eigen::Matrix6d& stiffness,
                          Eigen::Matrix6d& damping) const = 0;

    virtual bool getCurrentPose(const std::string& end_effector, 
                                Eigen::Affine3d& base_T_ee) const = 0;

    virtual bool getPoseTarget(const std::string& end_effector, 
                       Eigen::Affine3d& base_T_ref) const = 0;

    /**
     * @brief If the specified task (arg #1) is performing a reaching motion,
     * returns the index of the waypoint that is currently being processed.
     * Otherwise, a negative value is returned.
     * @param Task name
     * @return A value between 0 and N-1 (-1 if task is not in reaching phase)
     */
    virtual int getCurrentSegmentId(const std::string& end_effector) const = 0;

    virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const = 0;
    
    virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, 
                          Eigen::Vector3d * base_vel_ref = nullptr,
                          Eigen::Vector3d * base_acc_ref = nullptr ) const = 0;
    
    virtual bool getReferencePosture(Eigen::VectorXd& qref) const = 0;      
    virtual bool getReferencePosture(JointNameMap& qref) const = 0;



    virtual ~CartesianInterface(){}
    
};

} }

#endif

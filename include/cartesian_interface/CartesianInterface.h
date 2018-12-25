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

namespace XBot { namespace Cartesian {
    
    
/**
* @brief Enum describing a state for each task. Available states are:
*  - State::Online: the task is following an online getPoseReference
*  - State::Reacing: the task is performing a point-to-point motion
*/
enum class State { Reaching, Online };

/**
* @brief Enum describing a control mode for each task. Available values are:
*  - ControlType::Position: the task is following position references
*  - ControlType::Velocity: the task is following velocity references
*  - ControlType::Disabled: the task is disabled
*/
enum class ControlType { Position, Velocity, Disabled };

/**
* @brief The CartesianInterface class provides a generic way 
* to perform cartesian control of floating base robots.
* 
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
     * @param w_T_new_world New world pose w.r.t. old world
     */
    virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world) = 0;
    
    virtual const std::vector<std::string>& getTaskList() const = 0;
    virtual const std::string& getBaseLink(const std::string& ee_name) const = 0;
    virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type) = 0;
    virtual ControlType getControlMode(const std::string& ee_name) const = 0;
    virtual State getTaskState(const std::string& end_effector) const = 0;
    virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link) = 0;
    
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
    
    virtual void getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const = 0;
    virtual void getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const = 0;
    virtual void setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang) = 0;
    virtual void setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang) = 0;
    
    
    /* Online control */

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
                          
    virtual bool getCurrentPose(const std::string& end_effector, 
                                Eigen::Affine3d& base_T_ee) const = 0;
                          
    virtual bool getPoseTarget(const std::string& end_effector, 
                       Eigen::Affine3d& base_T_ref) const = 0;
                       
    virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const = 0;
    
    virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, 
                          Eigen::Vector3d * base_vel_ref = nullptr,
                          Eigen::Vector3d * base_acc_ref = nullptr ) const = 0;
    
    virtual bool getReferencePosture(Eigen::VectorXd& qref) const = 0;      
    virtual bool getReferencePosture(JointNameMap& qref) const = 0;
                          
    virtual ~CartesianInterface(){}
    
    static std::string ControlTypeAsString(ControlType ctrl);
    static std::string StateAsString(State state);
    
    static ControlType ControlTypeFromString(const std::string& ctrl);
    static State StateFromString(const std::string& state);
    
};

} }

#endif
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

namespace XBot { namespace Cartesian {

/**
* @brief The CartesianInterface class provides a generic way 
* to perform cartesian control of floating base robots.
* 
*/
class CartesianInterface
{

public:

    typedef std::shared_ptr<CartesianInterface> Ptr;
    typedef std::shared_ptr<const CartesianInterface> ConstPtr;
    
    enum class State { Reaching, Online };
    enum class ControlType { Position, Velocity, Disabled };
    
    CartesianInterface() = default;
    CartesianInterface(const CartesianInterface& other) = delete;
    CartesianInterface(const CartesianInterface&& other) = delete;
    CartesianInterface& operator=(const CartesianInterface& rhs) = delete;
    CartesianInterface& operator=(const CartesianInterface&& rhs) = delete;
    
    virtual bool reset() = 0;
    virtual bool update(double time, double period) = 0;
    
    virtual const std::vector<std::string>& getTaskList() const = 0;
    virtual const std::string& getBaseLink(const std::string& ee_name) const = 0;
    
    virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type) = 0;
    virtual ControlType getControlMode(const std::string& ee_name) const = 0;
    
    /* Point-to-point control */
    
    virtual bool setTargetPose(const std::string& end_effector, 
                       const Eigen::Affine3d& base_T_ref, 
                       double time = 0
                      ) = 0;
    
    virtual bool setTargetPosition(const std::string& end_effector, 
                           const Eigen::Vector3d& base_pos_ref, 
                           double time = 0) = 0;
    
    virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, 
                              double time = 0) = 0;
    
    virtual bool setTargetOrientation(const std::string& end_effector, 
                              const Eigen::Vector3d& base_pos_ref, 
                              double time = 0) = 0;
    
    virtual bool setTargetOrientation(const std::string& end_effector, 
                              const std::string& base_frame, 
                              const Eigen::Matrix3d& base_R_ref, 
                              double time = 0) = 0;
    
    virtual bool abort(const std::string& end_effector) = 0;
    
    
    
    
    /* Online control */

    virtual bool setPoseReference(const std::string& end_effector, 
                          const Eigen::Affine3d& base_T_ref, 
                          const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(),
                          const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero()) = 0;
    
    virtual bool setPositionReference(const std::string& end_effector, 
                              const Eigen::Vector3d& base_pos_ref, 
                              const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(),
                              const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero()) = 0;
    
    virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref, 
                                 const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(),
                                 const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero()) = 0;
    
    /* Monitoring */
    
    virtual bool getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref = nullptr,
                          Eigen::Vector6d * base_acc_ref = nullptr) const = 0;
                          
    virtual bool getCurrentPose(const std::string& end_effector, 
                                Eigen::Affine3d& base_T_ee) const = 0;
                          
    virtual bool getPoseTarget(const std::string& end_effector, 
                       Eigen::Affine3d& base_T_ref) const = 0;
                       
    virtual State getTaskState(const std::string& end_effector) const = 0;
    
    
    virtual ~CartesianInterface(){}
    
protected:
    
    
private:

    
    
};

} }

#endif
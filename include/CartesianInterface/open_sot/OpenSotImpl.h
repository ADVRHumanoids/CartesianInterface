#ifndef __XBOT_CARTESIAN_OPENSOT_IMPL_H__
#define __XBOT_CARTESIAN_OPENSOT_IMPL_H__


#include <CartesianInterface/CartesianInterfaceImpl.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/CoM.h>
#include <OpenSoT/tasks/velocity/Postural.h>

namespace XBot { namespace Cartesian {

class OpenSotImpl : public CartesianInterfaceImpl
{
    
public:
    
    OpenSotImpl();
        
    virtual bool setComPositionReference(const Eigen::Vector3d& w_com_ref, 
                                         const Eigen::Vector3d& w_vel_ref = Eigen::Vector3d::Zero(), 
                                         const Eigen::Vector3d& w_acc_ref = Eigen::Vector3d::Zero(), 
                                         ControlType control_type = ControlType::Position);

    virtual bool setPoseReference(const std::string& end_effector, 
                                  const Eigen::Affine3d& w_T_ref, 
                                  const Eigen::Vector6d& w_vel_ref = Eigen::Vector6d::Zero(), 
                                  const Eigen::Vector6d& w_acc_ref = Eigen::Vector6d::Zero(), 
                                  ControlType control_type = ControlType::Position);

    virtual bool update(double time, double period);

    virtual ~OpenSotImpl();
    
    
protected:
    
    
private:
    
    
};


} }

#endif
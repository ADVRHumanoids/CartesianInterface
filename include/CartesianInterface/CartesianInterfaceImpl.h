#ifndef __XBOT_CARTESIAN_IMPL_H__
#define __XBOT_CARTESIAN_IMPL_H__


#include <CartesianInterface/CartesianInterface.h>


namespace XBot { namespace Cartesian {

class CartesianInterfaceImpl : public CartesianInterface 
{
    
public:
    
    CartesianInterfaceImpl();
    
    virtual bool getPoseReference(const std::string& end_effector, 
                                Eigen::Affine3d& w_T_ref, 
                                Eigen::Vector6d& w_vel_ref, 
                                Eigen::Vector6d& w_acc_ref) const;

    virtual bool getPoseTarget(const std::string& end_effector, 
                            Eigen::Affine3d& w_T_ref) const;

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


    virtual bool abort(const std::string& end_effector);


    
protected:
    
    
private:
    
    
};


} }


#endif
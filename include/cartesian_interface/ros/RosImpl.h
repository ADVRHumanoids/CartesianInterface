#ifndef __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__
#define __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__

#include <cartesian_interface/CartesianInterface.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>


namespace XBot { namespace Cartesian {

    class RosImpl : public CartesianInterface 
    {
        
    public:
        
        RosImpl();
        virtual ~RosImpl();
        
        virtual bool reset();
        virtual const std::string& getBaseLink(const std::string& ee_name) const;
        virtual const std::vector< std::string >& getTaskList() const;
        virtual State getTaskState(const std::string& end_effector) const;
        virtual ControlType getControlMode(const std::string& ee_name) const;
        virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type);

        virtual bool getPoseTarget(const std::string& end_effector, 
                                   Eigen::Affine3d& base_T_ref) const;
                                   
        virtual bool getCurrentPose(const std::string& end_effector, 
                                    Eigen::Affine3d& base_T_ee) const;
                                    
        virtual bool getPoseReference(const std::string& end_effector, 
                                      Eigen::Affine3d& base_T_ref, 
                                      Eigen::Vector6d* base_vel_ref = nullptr, 
                                      Eigen::Vector6d* base_acc_ref = nullptr) const;

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
        
        virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, 
                                          double time = 0);
        
        virtual bool setTargetOrientation(const std::string& end_effector, 
                                          const std::string& base_frame, 
                                          const Eigen::Matrix3d& base_R_ref, 
                                          double time = 0);
        
        virtual bool setTargetOrientation(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time = 0);
        virtual bool setTargetPose(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, double time = 0);
        virtual bool setTargetPosition(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time = 0);

        virtual bool abort(const std::string& end_effector);
        virtual bool update(double time, double period);
            
    private:
        
        
        typedef actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ActionClient;
        std::vector<ActionClient> _action_client;
        
        std::vector<ros::Publisher> _pose_ref_pub;
        std::vector<ros::Subscriber> _pose_ref_sub;
        std::vector<ros::Subscriber> _pose_state_sub;
        ros::ServiceClient _get_ctrl_mode_srv, _task_state_srv;
        
    };


} }

#endif

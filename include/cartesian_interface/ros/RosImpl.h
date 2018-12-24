#ifndef __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__
#define __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__

#include <cartesian_interface/CartesianInterface.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

namespace XBot { namespace Cartesian {
    
    class RosImpl : public CartesianInterface 
    {
        
    public:
        
        friend std::ostream& operator<<(std::ostream& os, const RosImpl& r);
        
        RosImpl(std::string node_namespace = "");
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
                                      
        virtual bool getPoseReferenceRaw(const std::string& end_effector, 
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
        
        virtual bool setPoseReferenceRaw(const std::string& end_effector, 
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
        virtual bool setTargetPose(const std::string& end_effector, 
                                   const Eigen::Affine3d& base_T_ref, 
                                   double time = 0);
        bool setTargetPose(const std::string& end_effector, 
                           const Eigen::Affine3d& base_T_ref, 
                           double time,
                           bool incremental
                           );
        virtual bool setTargetPosition(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time = 0);

        virtual void getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const;
        virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, Eigen::Vector3d* base_vel_ref = nullptr, Eigen::Vector3d* base_acc_ref = nullptr) const;
        virtual bool getReferencePosture(XBot::JointNameMap& qref) const;
        virtual bool getReferencePosture(Eigen::VectorXd& qref) const;
        virtual void getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const;
        virtual void setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang);
        virtual void setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang);
        virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
        virtual bool setReferencePosture(const XBot::JointNameMap& qref);
        virtual bool setWayPoints(const std::string& end_effector, 
                                  const Trajectory::WayPointVector& way_points);
        bool setWayPoints(const std::string& end_effector, 
                                  const Trajectory::WayPointVector& way_points, 
                                  bool incremental
                                 );
        virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const;
        virtual bool reset(double time);
        virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);

        void loadController(const std::string& controller_name);
        bool waitReachCompleted(const std::string& ee_name, double timeout_sec = 0.0);
        virtual bool abort(const std::string& end_effector);
        virtual bool update(double time, double period);
            
    private:
        
        
        typedef actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ActionClient;
        
        class RosTask
        {
            
        private:
            
            ros::Subscriber state_sub;
            ros::Publisher ref_pub;
            ros::Publisher vref_pub;
            ActionClient reach_action;
            
            ros::ServiceClient get_prop_srv;
            ros::ServiceClient set_prop_srv;
            
            std::string distal_link;
            Eigen::Affine3d state;
            
            void state_callback(const geometry_msgs::PoseStampedConstPtr& msg);
            bool valid_state;
            
        public:
            
            typedef std::shared_ptr<RosTask> Ptr;
            
            RosTask(ros::NodeHandle nh, std::string distal_link);
            bool is_valid() const;
            
            Eigen::Affine3d get_state() const;
            void send_ref(const Eigen::Affine3d& ref);
            void send_vref(const Eigen::Vector6d& vref);
            void send_waypoints(const Trajectory::WayPointVector& wp, bool incremental = false);
            void get_properties(std::string& base_link, ControlType& ctrl_type);
            void set_base_link(const std::string& base_link);
            void set_ctrl_mode(ControlType ctrl_type);
            bool wait_for_result(ros::Duration timeout);
            
        };
        
        struct RosInitHelper
        {
            RosInitHelper(std::string node_namespace)
            {
                if(!ros::ok())
                {
                    std::string ns_arg = "__ns:=";
                    ns_arg += node_namespace;
                    std::vector<const char *> args {"", ns_arg.c_str()};
                    
                    int argc = args.size();
                    
                    ros::init(argc, (char **)args.data(), "cartesio_ros", 
                              ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
                    ROS_WARN("Initializing roscpp under namespace %s with anonymous name %s", 
                             ros::this_node::getNamespace().c_str(),
                             ros::this_node::getName().c_str()
                            );
                }
            }
        };
        
        void construct_from_tasklist();
        RosTask::Ptr get_task(const std::string& task_name, bool no_throw = true) const;
        
        RosInitHelper _ros_init_helper;
        
        ros::NodeHandle _nh;
        ros::CallbackQueue _queue;
        
        ros::ServiceClient _upd_limits_srv;
        ros::ServiceClient _load_ctrl_srv;
        ros::Publisher _posture_pub;
        std::map<std::string, RosTask::Ptr> _task_map;
        
        
        
        mutable ros::ServiceClient _tasklist_srv;        
        mutable std::vector<std::string> _tasklist;
    };

    std::ostream& operator<<(std::ostream& os, const RosImpl& r);

} }

#endif






















#ifndef __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__
#define __XBOT_CARTESIAN_INTERFACE_ROSIMPL_H__

#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cartesian_interface/ReachPoseAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <cartesian_interface/CartesianInterface.h>


namespace XBot { namespace Cartesian {
    
    class RosImpl : public CartesianInterface 
    {
        
    public:
        
        friend std::ostream& operator<<(std::ostream& os, const RosImpl& r);
        
        RosImpl(std::string ns = "cartesian");
        virtual ~RosImpl();
        
        virtual bool reset();
        virtual const std::string& getBaseLink(const std::string& ee_name) const;
        virtual TaskInterface getTaskInterface(const std::string& end_effector) const;
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
                                      
        virtual bool getDesiredInteraction(const std::string& end_effector, 
                          Eigen::Vector6d& force, 
                          Eigen::Matrix6d& stiffness,
                          Eigen::Matrix6d& damping) const;
                                      
        virtual bool setForceReference(const std::string& end_effector,
                                       const Eigen::Vector6d& force);
                                   
        virtual bool setDesiredStiffness(const std::string& end_effector,
                                         const Eigen::Matrix6d& k);
                                    
        virtual bool setDesiredDamping(const std::string& end_effector,
                                       const Eigen::Matrix6d& d);

        virtual bool setPoseReference(const std::string& end_effector, 
                          const Eigen::Affine3d& base_T_ref);
                            
        virtual bool setVelocityReference(const std::string& end_effector, 
                            const Eigen::Vector6d& base_vel_ref);
                            
        virtual bool setPoseReferenceRaw(const std::string& end_effector, 
                                const Eigen::Affine3d& base_T_ref);
        
        virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref);
                                    
        virtual bool setComVelocityReference(const Eigen::Vector3d& base_vel_ref);
        
        virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, 
                                          double time);
        
        virtual bool setTargetPose(const std::string& end_effector, 
                                   const Eigen::Affine3d& base_T_ref, 
                                   double time);
        
        bool setTargetPose(const std::string& end_effector, 
                           const Eigen::Affine3d& base_T_ref, 
                           double time,
                           bool incremental
                           );

        virtual void getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const;
        virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref, Eigen::Vector3d* base_vel_ref = nullptr, Eigen::Vector3d* base_acc_ref = nullptr) const;
        virtual bool getReferencePosture(XBot::JointNameMap& qref) const;
        virtual bool getReferencePosture(Eigen::VectorXd& qref) const;
        virtual void getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const;
        virtual void setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang);
        virtual void setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang);
        virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
        bool resetWorld(const std::string& ee_name);
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

        void loadController(const std::string& controller_name, 
                            const std::string& problem_description_name = "",
                            const std::string& problem_description_string = "",
                            const bool force_reload = true);
        bool waitReachCompleted(const std::string& ee_name, double timeout_sec = 0);
        bool setVelocityReferenceAsync(const std::string& ee_name, 
                                       const Eigen::Vector6d& vref,
                                       double timeout_sec
                                      );
        bool stopVelocityReferenceAsync(const std::string& ee_name);
        virtual bool abort(const std::string& end_effector);
        virtual bool update(double time, double period);

        bool getPoseFromTf(const std::string& source_frame, const std::string& target_frame, Eigen::Affine3d& t_T_s);
            
    private:
        
        
        typedef actionlib::SimpleActionClient<cartesian_interface::ReachPoseAction> ActionClient;
        
        class VelocityPublisherAsync
        {
            
        public:
            
            VelocityPublisherAsync(std::string topic_name, ros::Duration period);
            void start(ros::Duration timeout = ros::Duration(-1));
            void stop();
            void set_vref(const Eigen::Vector6d& vref);
            
        private:
            
            void timer_callback(const ros::TimerEvent& ev);
            
            ros::CallbackQueue _queue;
            std::recursive_mutex _mutex;
            ros::Publisher _vref_pub;
            ros::AsyncSpinner _spinner;
            ros::Timer _timer;
            ros::Time _timeout;
            
            
            geometry_msgs::TwistStamped _twist;
            
        };
        
        class RosTask
        {
            
        private:
            
            static const int VREF_ASYNC_PERIOD = 0.05;
            
            ros::Subscriber state_sub;
            ros::Publisher ref_pub;
            ros::Publisher vref_pub;
            ActionClient reach_action;
            
            VelocityPublisherAsync vref_async;
            
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
            void send_vref_async(const Eigen::Vector6d& vref, double timeout_duration);
            void stop_vref_async();
            void send_waypoints(const Trajectory::WayPointVector& wp, bool incremental = false);
            void get_properties(std::string& base_link, 
                                ControlType& ctrl_type,
                                TaskInterface& ifc_type);
            void set_base_link(const std::string& base_link);
            void set_ctrl_mode(ControlType ctrl_type);
            bool wait_for_result(ros::Duration timeout);
            void abort();
            
            const std::string& get_distal_link() const;
            
        };
        
        class RosInteractionTask : public RosTask
        {
        public:
            
            typedef std::shared_ptr<RosInteractionTask> Ptr;
            
            RosInteractionTask(ros::NodeHandle nh, std::string distal_link);
            
            void send_force(const Eigen::Vector6d& f);
            void send_impedance(const Eigen::Matrix6d& k, 
                                const Eigen::Matrix6d& d);
            
            void get_impedance(Eigen::Matrix6d& k,
                               Eigen::Matrix6d& d) const;
        private:
            
            Eigen::Matrix6d k, d;
            
            ros::Publisher impedance_pub;
            ros::Publisher force_pub;
            mutable ros::ServiceClient get_imp_srv;
            
        };
        
        struct RosInitHelper
        {
            RosInitHelper(std::string node_namespace);
        };
        
        void construct_from_tasklist();
        RosTask::Ptr get_task(const std::string& task_name, bool no_throw = true) const;
        RosInteractionTask::Ptr get_interaction_task(const std::string& task_name, bool no_throw = true) const;
        
        RosInitHelper _ros_init_helper;
        
        ros::NodeHandle _nh;
        ros::CallbackQueue _queue;
        tf::TransformListener _listener;
        
        ros::ServiceClient _upd_limits_srv;
        ros::ServiceClient _load_ctrl_srv;
        ros::Publisher _posture_pub;
        std::map<std::string, RosTask::Ptr> _task_map;
        std::map<std::string, RosInteractionTask::Ptr> _inter_task_map;
        
        
        
        mutable ros::ServiceClient _tasklist_srv;        
        mutable std::vector<std::string> _tasklist;
    };

    std::ostream& operator<<(std::ostream& os, const RosImpl& r);

} }

#endif
















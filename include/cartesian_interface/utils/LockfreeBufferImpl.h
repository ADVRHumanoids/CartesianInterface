#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/utils/boost/spsc_queue_ci.hpp>

namespace XBot { namespace Cartesian {
  
    class LockfreeBufferImpl : public CartesianInterface
    {
        
        typedef std::function<void(CartesianInterface*)> CallbackType;
      
    public:
        
        typedef std::shared_ptr<LockfreeBufferImpl> Ptr;
        
        LockfreeBufferImpl(CartesianInterface const * ci, ModelInterface::Ptr model);
        
        void callAvailable(CartesianInterface * ci);
        
        void pushState(CartesianInterface const * ci, ModelInterface const * model);

        void updateState();
        
        
        virtual bool setPoseReference(const std::string& end_effector, 
                                      const Eigen::Affine3d& base_T_ref, 
                                      const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(), 
                                      const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero());

        
        virtual bool setBaseLink(const std::string& ee_name, const std::string& new_base_link);

        
        virtual bool setControlMode(const std::string& ee_name, ControlType ctrl_type);

        
        virtual bool setWayPoints(const std::string& end_effector, 
                                  const Trajectory::WayPointVector& way_points);



        virtual bool abort(const std::string& end_effector);
        virtual void getAccelerationLimits(const std::string& ee_name, 
                                           double& max_acc_lin,
                                           double& max_acc_ang) const;
                                           
        virtual const std::string& getBaseLink(const std::string& ee_name) const;
        
        virtual bool getComPositionReference(Eigen::Vector3d& w_com_ref,
                                             Eigen::Vector3d* base_vel_ref = nullptr, 
                                             Eigen::Vector3d* base_acc_ref = nullptr) const;
                                             
        virtual ControlType getControlMode(const std::string& ee_name) const;
        
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
                                         
        virtual bool getPoseTarget(const std::string& end_effector, 
                                   Eigen::Affine3d& base_T_ref) const;
                                   
        virtual bool getReferencePosture(XBot::JointNameMap& qref) const;
        
        virtual bool getReferencePosture(Eigen::VectorXd& qref) const;
        
        virtual bool getTargetComPosition(Eigen::Vector3d& w_com_ref) const;
        
        virtual const std::vector< std::string >& getTaskList() const;
        
        virtual State getTaskState(const std::string& end_effector) const;
        
        virtual void getVelocityLimits(const std::string& ee_name,
                                       double& max_vel_lin, 
                                       double& max_vel_ang) const;
                                       
        virtual bool reset(double time);
        
        virtual bool resetWorld(const Eigen::Affine3d& w_T_new_world);
        
        virtual void setAccelerationLimits(const std::string& ee_name, 
                                           double max_acc_lin, 
                                           double max_acc_ang);
        
        virtual bool update(double time, double period);
        
        virtual bool setComPositionReference(const Eigen::Vector3d& base_com_ref, const Eigen::Vector3d& base_vel_ref = Eigen::Vector3d::Zero(), const Eigen::Vector3d& base_acc_ref = Eigen::Vector3d::Zero());
        
        virtual bool setReferencePosture(const JointNameMap& qref);
        
        virtual bool setPoseReferenceRaw(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, const Eigen::Vector6d& base_vel_ref = Eigen::Vector6d::Zero(), const Eigen::Vector6d& base_acc_ref = Eigen::Vector6d::Zero());
        
        virtual bool setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time = 0);
        
        virtual bool setTargetPose(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, double time = 0);
        
        virtual void setVelocityLimits(const std::string& ee_name, 
                                       double max_vel_lin, 
                                       double max_vel_ang);
    
    private:
        
        static const int CALL_QUEUE_SIZE = 128;
        static const int DEFAULT_QUEUE_SIZE = 10;
        
        template <typename T, int N = DEFAULT_QUEUE_SIZE>
        using LockFreeQueue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<N>>;
        
        struct TaskState 
        {
            std::string base_frame;
            
            Eigen::Affine3d T, Totg;
            Eigen::Vector6d vel;
            Eigen::Vector6d acc;
            
            std::pair<double,double> maxvel, maxacc;
            
            ControlType control_type;
            State state;
        };
        
        
        LockFreeQueue<CallbackType, CALL_QUEUE_SIZE> _call_queue;
        
        TaskState _task_tmp;
        Eigen::VectorXd _q_tmp;
        
        std::map<std::string, LockFreeQueue<TaskState>> _taskstate_queue_map;
        LockFreeQueue<Eigen::VectorXd> _model_state_queue;
        
        Eigen::VectorXd _q_tmp_read;
        std::map<std::string, TaskState> _taskstate_map;
        ModelInterface::Ptr _model;
        
        std::vector<std::string> _tasklist;
        
        
    };
    
    
} }
#ifndef __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__
#define __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__

#include <XBotInterface/ModelInterface.h>
#include <algorithm>
#include <lapack_svd/lapack_svd.h>


namespace XBot { namespace Cartesian { namespace Utils {
    
    class ForceEstimation
    {
      
    public:
        
        typedef std::shared_ptr<ForceEstimation> Ptr;
        
        static constexpr double DEFAULT_SVD_THRESHOLD = 0.05;

        ForceEstimation(ModelInterface::ConstPtr model, 
                        double svd_threshold = DEFAULT_SVD_THRESHOLD);
        
        ForceTorqueSensor::ConstPtr add_link(std::string name, 
                                             std::vector<int> dofs = {}, 
                                             std::vector<std::string> chains = {});
        
        void update();
        
        virtual void log(MatLogger::Ptr logger) const;
        
        ~ForceEstimation();
        
    protected:
        
        ModelInterface::ConstPtr _model;
        
        Eigen::VectorXd _y, _tau, _g;
    
    private:
        
        void compute_A_b();
        void solve();
        
        virtual void compute_residual();
        void allocate_workspace();
                
        
        struct TaskInfo
        {
            ForceTorqueSensor::Ptr sensor;
            std::vector<int> dofs;
            std::string link_name;
            Eigen::Vector6d wrench;
            Eigen::Matrix3d s_R_w;
            
        };
        Eigen::Vector6d _local_wrench;
        
        Eigen::MatrixXd _Jtot;
        Eigen::MatrixXd _A;
        Eigen::MatrixXd _Jtmp;
        Eigen::VectorXd _b, _sol;
        
        std::vector<TaskInfo> _tasks;
        std::set<int> _meas_idx;
        int _ndofs; 
        
        LapackSvd _svd;
        
        double _svd_th;
        
//         XBot::MatLogger::Ptr _logger;
        
    };   
    
    
    class ForceEstimationMomentumBased : public ForceEstimation
    {
      
    public:
        
        typedef std::shared_ptr<ForceEstimationMomentumBased> Ptr;
        
        static constexpr double DEFAULT_OBS_BW = 4.0;

        ForceEstimationMomentumBased(ModelInterface::ConstPtr model, 
                        double rate,
                        double obs_bw = DEFAULT_OBS_BW,
                        double svd_threshold = DEFAULT_SVD_THRESHOLD);
        
        void init_momentum_obs();

        bool get_residual(Eigen::VectorXd &res) const;
        bool get_static_residual(Eigen::VectorXd &static_res) const;
        
        void log(MatLogger::Ptr logger) const override;
        
    private:
        
        void compute_residual() override;

        double _rate, _k_obs;
        
        Eigen::VectorXd _p0, _p1, _p2, _qdot, _h, _coriolis, _y_static;
        Eigen::MatrixXd _M, _M_old, _Mdot;
        
    };
    
} } }




#endif

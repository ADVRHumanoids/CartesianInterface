#ifndef __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__
#define __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__

#include <XBotInterface/ModelInterface.h>
#include <algorithm>

namespace XBot { namespace Cartesian { namespace Utils {
    
    class ForceEstimation
    {
      
    public:
        
        typedef std::shared_ptr<ForceEstimation> Ptr;
        
        static constexpr double DEFAULT_SVD_THRESHOLD = 0.05;
	static constexpr double DEFAULT_RATE = 200.0;
	static constexpr double DEFAULT_OBS_BW = 4.0;

        ForceEstimation(ModelInterface::ConstPtr model, 
                        double svd_threshold = DEFAULT_SVD_THRESHOLD,
			double rate = DEFAULT_RATE,
			bool momentum_based = false,
			double obs_bw = DEFAULT_OBS_BW);
        
        ForceTorqueSensor::ConstPtr add_link(std::string name, 
                                             std::vector<int> dofs = {}, 
                                             std::vector<std::string> chains = {});
        
        void update();
        
        void log(MatLogger::Ptr logger) const;

        bool get_residuals(Eigen::VectorXd &res) const;
	bool get_static_residuals(Eigen::VectorXd &static_res) const;
	
	~ForceEstimation();
        
    private:
        
        void compute_A_b();
        void solve();
	
	void compute_residuals();
	void init_momentum_obs();
        
        struct TaskInfo
        {
            ForceTorqueSensor::Ptr sensor;
            std::vector<int> dofs;
            std::string link_name;
            
        };
        
        ModelInterface::ConstPtr _model;
        
        Eigen::MatrixXd _Jtot;
        Eigen::MatrixXd _A;
        Eigen::MatrixXd _Jtmp;
        
        Eigen::VectorXd _y, _tau, _g, _b, _sol;
        
        std::vector<TaskInfo> _tasks;
        std::set<int> _meas_idx;
        int _ndofs; 
        
        Eigen::JacobiSVD<Eigen::MatrixXd> _svd;
	
	double _rate, _k_obs;
	bool _momentum_based;
	
	Eigen::VectorXd _p0, _p1, _p2, _q, _qdot, _q_old, _h, _coriolis, _y_static;
	Eigen::MatrixXd _M, _M_old, _Mdot;
	
	XBot::MatLogger::Ptr _logger;
        
    };   
    
    
    
} } }




#endif

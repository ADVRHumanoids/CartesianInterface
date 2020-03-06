#ifndef __LCARTESIAN_INTERFACE_UTILS_FORCE_EST_H__
#define __LCARTESIAN_INTERFACE_UTILS_FORCE_EST_H__

#include <XBotInterface/ModelInterface.h>
#include <algorithm>

namespace XBot { namespace Cartesian { namespace Utils {
    
    class TorqueEstimation
    {
      
    public:
        
        typedef std::shared_ptr<TorqueEstimation> Ptr;
        
        static constexpr double DEFAULT_RATE = 200.0;
        static constexpr double DEFAULT_OBS_BW = 4.0;

        TorqueEstimation(ModelInterface::ConstPtr model, 
                        bool momentum_based = false,
                        double obs_bw = DEFAULT_OBS_BW);
        
        void update(double rate = Utils::TorqueEstimation::DEFAULT_RATE);
        
        void log(MatLogger::Ptr logger) const;

        void init_momentum_obs();

        bool get_residuals(Eigen::VectorXd &res) const;
        bool get_static_residuals(Eigen::VectorXd &static_res) const;
        
        ~TorqueEstimation();
        
    private:
        
        void compute_residuals(double rate);
        
        ModelInterface::ConstPtr _model;
        
        Eigen::VectorXd _y, _tau, _g;
        
        double _k_obs;
        bool _momentum_based;
        
        Eigen::VectorXd _p0, _p1, _p2, _q, _qdot, _h, _coriolis, _y_static;
        Eigen::MatrixXd _M, _M_old, _Mdot;
        
        XBot::MatLogger::Ptr _logger;
        
    };   
    
    
    
} } }




#endif

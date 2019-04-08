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

        ForceEstimation(ModelInterface::ConstPtr model, 
                        double svd_threshold = DEFAULT_SVD_THRESHOLD);
        
        ForceTorqueSensor::ConstPtr add_link(std::string name, 
                                             std::vector<int> dofs = {}, 
                                             std::vector<std::string> chains = {});
        
        void update();
        
        void log(MatLogger::Ptr logger) const;
        
        
    private:
        
        void compute_A_b();
        void solve();
        
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
        
    };   
    
    
    
} } }




#endif

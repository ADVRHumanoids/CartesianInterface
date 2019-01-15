#ifndef __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__
#define __CARTESIAN_INTERFACE_UTILS_FORCE_EST_H__

#include <XBotInterface/ModelInterface.h>
#include <algorithm>
#include <OpenSoT/solvers/iHQP.h>
#include <OpenSoT/tasks/GenericTask.h>

namespace XBot { namespace Cartesian { namespace Utils {
    
    class ForceEstimation
    {
      
    public:
        
        typedef std::shared_ptr<ForceEstimation> Ptr;

        ForceEstimation(ModelInterface::ConstPtr model);
        
        ForceTorqueSensor::ConstPtr add_link(std::string name, 
                                             std::vector<int> dofs = {}, 
                                             std::vector<std::string> chains = {});
        
        void update();
        
        void log(MatLogger::Ptr logger) const;
        
        
    private:
        
        void compute_A_b();
        bool solve();
        
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

        OpenSoT::tasks::GenericTask::Ptr _generic_task;
        OpenSoT::solvers::iHQP::Ptr _solver;

        std::string _id;

        bool _solver_inited;
        
    };   
    
    
    
} } }




#endif

#ifndef __CI_MANIPULABILITY_H__
#define __CI_MANIPULABILITY_H__

#include <cartesian_interface/ProblemDescription.h>
#include <cartesian_interface/utils/LoadConfig.h>

#include <XBotInterface/MatLogger.hpp>
#include <ros/ros.h>

#include <functional>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>

namespace XBot { namespace Cartesian {
        
class ManipulabilityAnalyzer
{
public:
    
    ManipulabilityAnalyzer(ModelInterface::ConstPtr model, 
                           ProblemDescription ik_problem, 
                           std::string tf_prefix
                          );
    
    void compute();
    
    static visualization_msgs::Marker ComputeEllipsoidFromJacobian(const Eigen::MatrixXd& J, 
                                                                   const Eigen::Vector3d& pos, 
                                                                   const std::string& base_link,
                                                                   int start_idx);
    
    static visualization_msgs::Marker ComputeEllipsoidFromQuadraticForm(const Eigen::MatrixXd& JJtinv, 
                                                                   const Eigen::Vector3d& pos, 
                                                                   const std::string& base_link,
                                                                   int start_idx);
    
private:
    
    bool compute_task_matrix(TaskDescription::Ptr task, Eigen::MatrixXd& A, Eigen::Affine3d& T);
    void publish();
    
    XBot::MatLogger::Ptr _logger;
    
    ModelInterface::ConstPtr _model;
    ProblemDescription _ik_problem;
    
    std::vector<Eigen::MatrixXd> _nullspace_bases, _tasks;
    std::map<std::string, Eigen::Affine3d> _task_poses;
    std::map<std::string, std::pair<int,int>> _task_idx;
    
    std::map<std::string, ros::Publisher> _pub_map_pos, _pub_map_rot;
    std::map<std::string, double> _scale_pos, _scale_rot;
    ros::NodeHandle _nhpriv;
    
    std::string _tf_prefix_slash;
    
    
};


} }






#endif


#include <cartesian_interface/utils/Manipulability.h>

#if EIGEN_VERSION_AT_LEAST(3, 2, 92)
    #define HAS_SVD_SET_THRESHOLD
#else
    #warning "Your Eigen version does not support SVD threshold. Manipulability analyzer may not be stable."
#endif


XBot::Cartesian::ManipulabilityAnalyzer::ManipulabilityAnalyzer(XBot::ModelInterface::ConstPtr model,
                                                                XBot::Cartesian::ProblemDescription ik_problem, 
                                                                std::string tf_prefix):
    _model(model),
    _ik_problem(ik_problem),
    _logger(XBot::MatLogger::getLogger("/tmp/manipulability_analyzer_log")),
    _nhpriv("~")
{
    _tf_prefix_slash = tf_prefix == "" ? "" : tf_prefix + "/";
    
    double master_scale_pos = _nhpriv.param("global_scale/linear", 1.0);
    double master_scale_rot = _nhpriv.param("global_scale/angular", 0.333);
    
    ros::NodeHandle nh("cartesian");
    for(int i = 0; i < ik_problem.getNumTasks(); i++)
    {
        for(int j = 0; j < ik_problem.getTask(i).size(); j++)
        {
            if(ik_problem.getTask(i).at(j)->type != TaskType::Cartesian)
            {
                continue;
            }
            
            std::string name = ik_problem.getTask(i).at(j)->name;
            _pub_map_pos[name] = nh.advertise<visualization_msgs::Marker>("ellipses/" + name + "/linear", 1);
            _pub_map_rot[name] = nh.advertise<visualization_msgs::Marker>("ellipses/" + name + "/angular", 1);
            
            _scale_pos[name] = _nhpriv.param("scale/" + name + "/linear", 1.0) * master_scale_pos;
            _scale_rot[name] = _nhpriv.param("scale/" + name + "/angular", 1.0) * master_scale_rot;
            
            
        }
    }
    
    
}


bool XBot::Cartesian::ManipulabilityAnalyzer::compute_task_matrix(XBot::Cartesian::TaskDescription::Ptr task, 
                                                                  Eigen::MatrixXd& A, 
                                                                  Eigen::Affine3d& T)
{
    CartesianTask::Ptr cart_ij = GetAsCartesian(task);
    ComTask::Ptr com_ij = GetAsCom(task);
    
    if(cart_ij)
    {
        
        if(cart_ij->base_link == "world")
        {
            _model->getJacobian(cart_ij->distal_link, A);
            _model->getPose(cart_ij->distal_link, T);
        }
        else
        {
            _model->getRelativeJacobian(cart_ij->distal_link, cart_ij->base_link, A);
            _model->getPose(cart_ij->distal_link, cart_ij->base_link, T);
        }
    }
    else if(com_ij)
    {
        _model->getCOMJacobian(A);
        A.conservativeResize(6, A.cols());
        A.bottomRows(3).setZero();
        Eigen::Vector3d compos;
        _model->getCOM(compos);
        T.setIdentity();
        T.translation() = compos;
    }
    else
    {
        return false;
    }
    
    for(int k = 0; k < 6; k++)
    {
        if( std::find(cart_ij->indices.begin(), cart_ij->indices.end(), k) == cart_ij->indices.end() )
        {
            A.row(k).setZero();
        }
    }
    
    return true;
}


void XBot::Cartesian::ManipulabilityAnalyzer::compute()
{
    _nullspace_bases.resize(_ik_problem.getNumTasks() + 1);
    _nullspace_bases[0] = Eigen::MatrixXd::Identity(_model->getJointNum(), _model->getJointNum());
    _tasks.resize(_ik_problem.getNumTasks());
    
    for(int i = 0; i < _ik_problem.getNumTasks(); i++)
    {
        Eigen::MatrixXd Ji(0, _nullspace_bases.at(i).cols()); 
        
        for(int j = 0; j < _ik_problem.getTask(i).size(); j++)
        {
            Eigen::MatrixXd Aij;
            Eigen::Affine3d Tij;
            TaskDescription::Ptr task = _ik_problem.getTask(i).at(j);
            if(!compute_task_matrix(task, Aij, Tij))
            {
                continue;
            }
            
            _task_poses[task->name] = Tij;
            _task_idx[task->name] = std::make_pair(i, static_cast<int>(Ji.rows()));
            
            Aij = Aij * _nullspace_bases.at(i);
            
            Ji.conservativeResize(Ji.rows() + 6, Ji.cols());
            Ji.bottomRows(6) = Aij;
            
            _logger->add("A_" + std::to_string(i) + "_" + std::to_string(j), Aij);
            
        }
        
        int dim = Ji.rows();
        
        _tasks[i] = (Ji*Ji.transpose() + 1e-6*Eigen::MatrixXd::Identity(dim,dim)).inverse();
        
        _logger->add("A_" + std::to_string(i), Ji);
        
        if(Ji.size() > 0)
        {
            auto svd_i = Ji.jacobiSvd(Eigen::ComputeFullV);
            
#ifdef HAS_SVD_SET_THRESHOLD
            svd_i.setThreshold(1e-6);
#endif
            
            int ns_dim = svd_i.matrixV().cols() - svd_i.nonzeroSingularValues();
            _nullspace_bases[i+1] = _nullspace_bases[i] * svd_i.matrixV().rightCols(ns_dim);
        }
        else
        {
            _nullspace_bases[i+1] = _nullspace_bases[i];
        }
        
        _logger->add("NS_" + std::to_string(i+1), _nullspace_bases[i+1]);
        
    }
    
    
    publish();
}

void XBot::Cartesian::ManipulabilityAnalyzer::publish()
{
    auto now = ros::Time::now();
    
    for(auto& pair : _pub_map_pos)
    {
        std::string task_name = pair.first;
        
        int prio = _task_idx.at(task_name).first;
        int start_idx = _task_idx.at(task_name).second;
        
        auto marker_pos = ComputeEllipsoidFromQuadraticForm(_tasks[prio], 
                                                            _task_poses.at(task_name).translation(), 
                                                            _tf_prefix_slash + "world_odom",
                                                            start_idx
                                                           );
        
        marker_pos.scale.x *= _scale_pos.at(task_name);
        marker_pos.scale.y *= _scale_pos.at(task_name);
        marker_pos.scale.z *= _scale_pos.at(task_name);
        
        auto marker_rot = ComputeEllipsoidFromQuadraticForm(_tasks[prio], 
                                                            _task_poses.at(task_name).translation(), 
                                                            _tf_prefix_slash + "world_odom",
                                                            start_idx + 3
                                                           );
        
        marker_rot.scale.x *= _scale_rot.at(task_name);
        marker_rot.scale.y *= _scale_rot.at(task_name);
        marker_rot.scale.z *= _scale_rot.at(task_name);
        
        pair.second.publish(marker_pos);
        _pub_map_rot.at(task_name).publish(marker_rot);
    }
}


visualization_msgs::Marker XBot::Cartesian::ManipulabilityAnalyzer::ComputeEllipsoidFromQuadraticForm(const Eigen::MatrixXd& JJtinv,
                                                                                                      const Eigen::Vector3d& pos, const std::string& base_link, int start_idx)
{
    static Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Matrix3d K = JJtinv.middleRows<3>(start_idx).middleCols<3>(start_idx);
    eigensolver.computeDirect(K);
    
    
    Eigen::Affine3d T_ell;
    T_ell.setIdentity();
    T_ell.translation() = pos;
    T_ell.linear() = eigensolver.eigenvectors();
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_link;
    marker.header.stamp = ros::Time::now();
    marker.ns = "ellipses";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    tf::poseEigenToMsg(T_ell, marker.pose);
    marker.scale.x = 1.0/std::sqrt(eigensolver.eigenvalues()(0));
    marker.scale.y = 1.0/std::sqrt(eigensolver.eigenvalues()(1));
    marker.scale.z = 1.0/std::sqrt(eigensolver.eigenvalues()(2));
    marker.color.a = 0.75; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    return marker;
}

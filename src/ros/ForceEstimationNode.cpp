#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <cartesian_interface/ForceEstimationMsg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/estimation/ForceEstimation.h>

int main(int argc, char ** argv)
{
    using namespace XBot::Cartesian;
    
    ros::init(argc, argv, "force_estimation_node");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");
    
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());
//     auto imu = robot->getImu().begin()->second;
    
    double rate = nh_priv.param("rate", 25.0);
    auto links = nh_priv.param("links", std::vector<std::string>());
    
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/links is empty, exiting..");
        return 0;
    }
    
    auto chains = nh_priv.param("chains", std::vector<std::string>());
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/chains is empty, will use all torque sensors");
    }
    
    double svd_th = nh_priv.param("svd_threshold", (double)Utils::ForceEstimation::DEFAULT_SVD_THRESHOLD);
    
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    
    bool momentum_based = nh_priv.param("use_momentum_based_observer", false);
    double obs_bw = nh_priv.param("obs_bw", 4.0);
    Utils::ForceEstimation f_est(model, svd_th, rate, true, obs_bw);
    
    std::map<XBot::ForceTorqueSensor::ConstPtr, ros::Publisher> ft_map;
    
    for(auto l : links)
    {
        auto dofs = nh_priv.param(l + "/dofs", std::vector<int>());
        
        auto pub = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/" + l, 1);
        
        ft_map[f_est.add_link(l, dofs, chains)] = pub;
    }
    
    auto res_pub = nh.advertise<cartesian_interface::ForceEstimationMsg>("residuals", 1);
    
    Eigen::VectorXd tau;
    Eigen::VectorXd res;
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
//         model->setFloatingBaseState(imu);
        model->update();
        model->getJointEffort(tau);
        tau += tau_offset;
        model->setJointEffort(tau);
        
        f_est.update();
	
	f_est.get_residuals(res);
	
	cartesian_interface::ForceEstimationMsg res_msg;
        
	res_msg.residuals.header.stamp = ros::Time::now();
	
	res_msg.residuals.name.reserve(model->getJointNum());
	res_msg.residuals.effort.reserve(model->getJointNum());
	int i = 0;
	for(const std::string& joint_name : model->getEnabledJointNames())
	{
	    res_msg.residuals.name.push_back(joint_name);
	    res_msg.residuals.effort.push_back(res[i]);
	    i++;
	}
	
        for(auto& ft_pub : ft_map)
        {
            geometry_msgs::WrenchStamped msg;
            
            Eigen::Vector6d wrench;
            ft_pub.first->getWrench(wrench);
            
            tf::wrenchEigenToMsg(wrench, msg.wrench);
            
            msg.header.frame_id = ft_pub.first->getSensorName();
            msg.header.stamp = ros::Time::now();
	    
	    res_msg.wrenches.push_back(msg);
            
            ft_pub.second.publish(msg);
        }
        
        res_pub.publish(res_msg);
        
        loop_rate.sleep();
    }
    
    return 0;
    
}

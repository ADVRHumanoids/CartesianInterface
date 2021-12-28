#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/estimation/ForceEstimation.h>
#include <matlogger2/matlogger2.h>

int main(int argc, char ** argv)
{
    using namespace XBot::Cartesian;
    
    // init ros, get handles
    ros::init(argc, argv, "force_estimation_node");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");
    
    // get robot, model, and one imu
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());
    auto imu = robot->getImu().begin()->second;
    
    // configure node
    double rate = nh_priv.param("rate", 100.0);
    auto links = nh_priv.param("links", std::vector<std::string>());
    
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/links is empty, exiting..");
        return 1;
    }
    
    auto chains = nh_priv.param("chains", std::vector<std::string>());
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/chains is empty, will use all torque sensors");
    }
    
    double svd_th = nh_priv.param("svd_threshold", (double)Utils::ForceEstimation::DEFAULT_SVD_THRESHOLD);

    // get torque offset map
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    
    // construct force estimation class
    Utils::ForceEstimation::Ptr f_est_ptr;

    if(nh_priv.param("use_momentum_based", false))
    {
        ROS_INFO("using momentum based estimation");
        f_est_ptr = std::make_shared<Utils::ForceEstimationMomentumBased>(model, rate, svd_th);
    }
    else
    {
        f_est_ptr = std::make_shared<Utils::ForceEstimation>(model, svd_th);
    }

    Utils::ForceEstimation& f_est = *f_est_ptr;

    // set ignored joints
    auto ignored_joints = nh_priv.param("ignored_joints", std::vector<std::string>());
    for(auto jname : ignored_joints)
    {
        f_est.setIgnoredJoint(jname);
    }
    
    // generate virtual fts
    std::map<XBot::ForceTorqueSensor::ConstPtr, ros::Publisher> ft_map;
    
    for(auto l : links)
    {
        auto dofs = nh_priv.param(l + "/dofs", std::vector<int>());
        
        auto pub = nh.advertise<geometry_msgs::WrenchStamped>("force_estimation/" + l, 1);
        
        ft_map[f_est.add_link(l, dofs, chains)] = pub;
    }

    // log
    XBot::MatLogger2::Ptr logger;
    if(nh_priv.param("enable_log", false))
    {
        logger = XBot::MatLogger2::MakeLogger("/tmp/force_estimation_log");
    }
    
    // start looping
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        // update model from robot, set imu
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        model->getJointEffort(tau);
        tau += tau_offset;
        model->setJointEffort(tau);
        
        // update force estimation
        f_est.update();
        
        // publish to topics
        for(auto& ft_pub : ft_map)
        {
            geometry_msgs::WrenchStamped msg;
            
            Eigen::Vector6d wrench;
            ft_pub.first->getWrench(wrench);
            
            tf::wrenchEigenToMsg(wrench, msg.wrench);
            
            msg.header.frame_id = ft_pub.first->getSensorName();
            msg.header.stamp = ros::Time::now();
            
            ft_pub.second.publish(msg);
        }

        // log
        if(logger)
        {
            f_est.log(logger);
        }
        
        // sync with desired loop freq
        loop_rate.sleep();
    }
    
}

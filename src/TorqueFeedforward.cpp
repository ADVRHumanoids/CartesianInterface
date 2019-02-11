#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;

void on_force_recv(const geometry_msgs::WrenchStamped& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg.wrench, g_fmap_ptr->at(l));
}

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "torque_feedforward_node");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");
    
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());
    auto imu = robot->getImu().begin()->second;
    
    robot->setControlMode(XBot::ControlMode::Effort());
    
    
    double rate = nh_priv.param("rate", 100.0);
    auto links = nh_priv.param("links", std::vector<std::string>());
    auto blacklist = nh_priv.param("blacklist", std::vector<std::string>());
    
    if(links.size() == 0)
    {
        ROS_INFO("Private parameter ~/links is empty, exiting..");
        return 0;
    }
    
    std::map<std::string, XBot::ControlMode> ctrl_map;
    for(auto l : blacklist)
    {
        if(!robot->hasJoint(l))
        {
            throw std::runtime_error("Joint '" + l + "' undefined");
        }
        
        ctrl_map[l] = XBot::ControlMode::Idle();
    }
    robot->setControlMode(ctrl_map);
    
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    
    std::map<std::string, ros::Subscriber> sub_map;
    std::map<std::string, Eigen::Vector6d> f_map;
    g_fmap_ptr = &f_map;
    
    for(auto l : links)
    {
        auto sub = nh.subscribe<geometry_msgs::WrenchStamped>("force_ffwd/" + l,
                                                              1, 
                                                              std::bind(on_force_recv, std::placeholders::_1, l));
        
        sub_map[l] = sub;
        f_map[l] = Eigen::Vector6d::Zero();
    }
    
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    while(ros::ok())
    {
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        
        model->computeGravityCompensation(tau);
        tau -= tau_offset;
        
        for(const auto& pair : f_map)
        {
            Eigen::MatrixXd J;
            Eigen::Matrix3d R;
            
            model->getJacobian(pair.first, J);
            model->getOrientation(pair.first, R);
            
            Eigen::Vector6d f_world = pair.second;
            f_world.head<3>() = R * f_world.head<3>();
            f_world.tail<3>() = R * f_world.tail<3>();
            
            tau += J.transpose() * f_world;
            
        }
        
        model->setJointEffort(tau);
        robot->setReferenceFrom(*model, XBot::Sync::Effort);
        robot->move();
        
        loop_rate.sleep();
    }
    
    return 0;
    
}

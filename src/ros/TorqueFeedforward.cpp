#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cartesian_interface/SetContactFrame.h>
#include <eigen_conversions/eigen_msg.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include <OpenSoT/utils/ForceOptimization.h>

std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
std::map<std::string, ros::Time> * g_timeoutmap_ptr;
const double FORCE_TTL = 0.1;
boost::shared_ptr<OpenSoT::utils::ForceOptimization> g_fopt;

void on_force_recv(const geometry_msgs::WrenchStampedConstPtr& msg, std::string l)
{
    tf::wrenchMsgToEigen(msg->wrench, g_fmap_ptr->at(l));
    g_timeoutmap_ptr->at(l) = ros::Time::now() + ros::Duration(FORCE_TTL);
}

bool on_contact_frame_changed(cartesian_interface::SetContactFrameRequest& req, 
                              cartesian_interface::SetContactFrameResponse& res)
{
    Eigen::Quaterniond q;
    
    tf::quaternionMsgToEigen(req.orientation, q);
    q.normalize();
    
    std::stringstream ss;
    ss << q.toRotationMatrix() << std::endl;
    
    g_fopt->setContactRotationMatrix(req.link_name, q.toRotationMatrix());
    
    res.success = true;
    res.message = "Successfully changed contact frame for link '" + req.link_name + "' to \n" + ss.str();
    
    return true;
}

int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "torque_feedforward");
    ros::NodeHandle nh("cartesian/torque_feedforward");
    ros::NodeHandle nh_priv("~");
    
    auto robot = XBot::RobotInterface::getRobot(XBot::ConfigOptionsFromParamServer());
    auto model = XBot::ModelInterface::getModel(XBot::ConfigOptionsFromParamServer());
    auto imu = robot->getImu().begin()->second;
    
    robot->setControlMode(XBot::ControlMode::Effort());
    
    
    double rate = nh_priv.param("rate", 100.0);
    auto ffwd_links = nh_priv.param("ffwd_links", std::vector<std::string>());
    auto contact_links = nh_priv.param("contact_links", std::vector<std::string>());
    auto blacklist = nh_priv.param("blacklist", std::vector<std::string>());
    bool optimize_contact_torque = nh_priv.param("optimize_contact_torque", false);
    
    /* Set joint blacklist */
    std::map<std::string, XBot::ControlMode> ctrl_map;
    for(auto j : blacklist)
    {
        if(!robot->hasJoint(j))
        {
            if(!robot->hasChain(j))
            {
                throw std::runtime_error("Joint or chain '" + j + "' undefined");
            }
            else
            {
                for(auto jc: robot->chain(j).getJointNames())
                {
                    ROS_INFO("Joint '%s' is blacklisted", jc.c_str());
                    ctrl_map[jc] = XBot::ControlMode::Idle();
                }
            }
            
        }
        
        ROS_INFO("Joint '%s' is blacklisted", j.c_str());
        ctrl_map[j] = XBot::ControlMode::Idle();
    }
    robot->setControlMode(ctrl_map);
    
    /* Torque offset */
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getJointNum());
    model->mapToEigen(tau_off_map_xbot, tau_offset);
    std::cout << "Torque offset: " << tau_offset.transpose() << std::endl;
    
    /* Subscribe to force ffwd topics */
    std::map<std::string, ros::Subscriber> sub_map;
    std::map<std::string, Eigen::Vector6d> f_map;
    std::map<std::string, ros::Time> timeout_map; 
    g_fmap_ptr = &f_map;
    g_timeoutmap_ptr = &timeout_map;
    
    for(auto l : ffwd_links)
    {
        auto sub = nh.subscribe<geometry_msgs::WrenchStamped>("force_ref/" + l,
                                                              1, 
                                                              boost::bind(on_force_recv, _1, l));
        
        sub_map[l] = sub;
        f_map[l] = Eigen::Vector6d::Zero();
        
        ROS_INFO("Subscribed to topic '%s'", sub.getTopic().c_str());
    }
    
    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);
    
    auto f_opt = g_fopt = boost::make_shared<OpenSoT::utils::ForceOptimization>(model, 
                                                                       contact_links, 
                                                                       optimize_contact_torque);
    
    std::vector<Eigen::Vector6d> forces;
    
    /* Service to change link orientation */
    auto contact_rot_srv = nh.advertiseService("change_contact_frame", 
                                               on_contact_frame_changed);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        /* Sense robot state and update model */
        robot->sense(false);
        model->syncFrom(*robot, XBot::Sync::All, XBot::Sync::MotorSide);
        model->setFloatingBaseState(imu);
        model->update();
        
        /* Compute gcomp */
        model->computeGravityCompensation(tau);
        tau -= tau_offset;
        
        /* Contact force distribution */
        f_opt->compute(tau, forces, tau);
        
        for(auto f : forces)
        {
            std::cout << f.transpose() << std::endl;
        }
        
        
        /* Check for expired force refs */
        for(const auto& pair : timeout_map)
        {
            if(ros::Time::now() > pair.second)
            {
                f_map.at(pair.first).setZero();
            }
        }
        
        /* Add force feedforward  term */
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
        
        /* Send torque to joints */
        model->setJointEffort(tau);
        robot->setReferenceFrom(*model, XBot::Sync::Effort);
        robot->move();
        
        loop_rate.sleep();
    }
    
    return 0;
    
}

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <cartesian_interface/SetContactFrame.h>
#include <eigen_conversions/eigen_msg.h>

#include <xbot2_interface/robotinterface2.h>
#include <xbot2_interface/ros/config_from_param.hpp>

#include "opensot/OpenSotUtils.h"
#include <OpenSoT/utils/ForceOptimization.h>

using namespace XBot::Cartesian;

std::map<std::string, Eigen::Vector6d> * g_fmap_ptr;
std::map<std::string, ros::Time> * g_timeoutmap_ptr;
const double FORCE_TTL = 0.1;
std::shared_ptr<OpenSoT::utils::ForceOptimization> g_fopt;

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
    Eigen::IOFormat print_fmt(2, 0, ", ", ";", "", "", "[", "]");
    ss << q.toRotationMatrix().format(print_fmt) << std::endl;

    g_fopt->setContactRotationMatrix(req.link_name, q.toRotationMatrix());

    res.success = true;
    res.message = "Successfully changed contact frame for link '" + req.link_name + "' to: " + ss.str();

    return true;
}

int main(int argc, char ** argv)
{
    // ros init, nodehandles
    ros::init(argc, argv, "torque_feedforward");
    ros::NodeHandle nh("cartesian/torque_feedforward");
    ros::NodeHandle nh_priv("~");

    // create robot and model
    auto robot = XBot::RobotInterface::getRobot(XBot::Utils::ConfigOptionsFromParamServer());
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(XBot::Utils::ConfigOptionsFromParamServer());

    // retrieve imu
    XBot::ImuSensor::ConstPtr imu;

    if(nh_priv.param("use_imu", true) && robot->getImu().size() > 0)
    {
        imu = robot->getImu().begin()->second;
    }
    else
    {
        Logger::warning("IMU not found: map is empty\n");
    }

    // set control mode
    robot->setControlMode(XBot::ControlMode::Effort());

    // set base orientation
    std::vector<double> base_rot = {0, 0, 0, 1};
    nh_priv.getParam("base_rot", base_rot);
    Eigen::Quaterniond base_quat(base_rot.data());
    std::cout << "setting base orientation.. \n";
    Eigen::Matrix3d w_R_base = base_quat.toRotationMatrix();
    if(model->setFloatingBaseOrientation(w_R_base))
    {
        std::cout << "set base orientation to " <<
                     base_quat.coeffs().transpose() << "\n";
    }
    model->update();

    // configure node
    double rate = nh_priv.param("rate", 100.0);
    auto ffwd_links = nh_priv.param("ffwd_links", std::vector<std::string>());
    auto contact_links = nh_priv.param("contact_links", std::vector<std::string>());
    auto blacklist = nh_priv.param("blacklist", std::vector<std::string>());
    bool optimize_contact_torque = nh_priv.param("optimize_contact_torque", false);
    double friction_coeff = nh_priv.param("friction_coeff", 0.5);

    /* Set joint blacklist */
    XBot::CtrlModeMap ctrl_map;
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
                for(auto jc: robot->getChain(j)->getJointNames())
                {
                    ROS_INFO("Joint '%s' is blacklisted", jc.c_str());
                    ctrl_map[jc] = XBot::ControlMode::None();
                }
            }

        }

        ROS_INFO("Joint '%s' is blacklisted", j.c_str());
        ctrl_map[j] = XBot::ControlMode::None();
    }
    robot->setControlMode(ctrl_map);

    /* Torque offset */
    auto tau_off_map = nh_priv.param("torque_offset", std::map<std::string, double>());
    XBot::JointNameMap tau_off_map_xbot(tau_off_map.begin(), tau_off_map.end());
    Eigen::VectorXd tau_offset;
    tau_offset.setZero(model->getNv());
    model->mapToV(tau_off_map_xbot, tau_offset);
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
        timeout_map[l] = ros::Time::now();

        ROS_INFO("Subscribed to topic '%s'", sub.getTopic().c_str());
    }

    /* Force distribution */
    std::vector<Eigen::Vector6d> forces;

    /* Publish optimized forces */
    std::vector<ros::Publisher> f_pubs;

    ros::ServiceServer contact_rot_srv;
    if(model->isFloatingBase() && nh_priv.param("force_opt", true))
    {
        auto f_opt = g_fopt = SotUtils::make_shared<OpenSoT::utils::ForceOptimization>(model,
                                                                       contact_links,
                                                                       optimize_contact_torque,
                                                                       friction_coeff
                                                                               );




        for(auto c : contact_links)
        {
            auto pub = nh.advertise<geometry_msgs::WrenchStamped>("optimized_force/" + c, 1);
            f_pubs.push_back(pub);
        }

        /* Service to change link orientation */
        contact_rot_srv = nh.advertiseService("change_contact_frame",
                                                on_contact_frame_changed);

    }

    Eigen::VectorXd tau;
    ros::Rate loop_rate(rate);

    while(ros::ok())
    {
        ros::spinOnce();

        /* Sense robot state and update model */
        robot->sense(false);
        model->syncFrom(*robot, XBot::ControlMode::ALL, XBot::Sync::MotorSide);

        if(model->isFloatingBase() && imu)
        {
            model->setFloatingBaseState(*imu);
            model->update();
        }

        /* Compute gcomp */
        model->computeGravityCompensation(tau);
        tau -= tau_offset;

        /* Contact force distribution */
        if(g_fopt)
        {
            g_fopt->compute(tau, forces, tau);

            for(int i = 0; i < forces.size(); i++)
            {
                geometry_msgs::WrenchStamped msg;
                msg.header.frame_id = contact_links[i];
                msg.header.stamp = ros::Time::now();

                Eigen::Matrix3d w_R_l;
                model->getOrientation(contact_links[i], w_R_l);

                auto f = forces[i];
                f.head<3>() = w_R_l.transpose() * f.head<3>();
                f.tail<3>() = w_R_l.transpose() * f.tail<3>();

                tf::wrenchEigenToMsg(f, msg.wrench);
                f_pubs[i].publish(msg);
            }

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
            f_world.head<3>() = f_world.head<3>();
            f_world.tail<3>() = f_world.tail<3>();

            tau += J.transpose() * f_world;

        }

        /* Send torque to joints */
        model->setJointEffort(tau);
        robot->setReferenceFrom(*model, XBot::ControlMode::EFFORT);
        robot->move();

        loop_rate.sleep();
    }

    return 0;

}

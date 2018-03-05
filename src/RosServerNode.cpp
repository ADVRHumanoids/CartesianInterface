#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/Utils.h>
#include <CartesianInterface/ros/RosServerClass.h>
#include <CartesianInterface/open_sot/OpenSotImpl.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

using namespace XBot::Cartesian;

int main(int argc, char **argv){
    
    XBot::Logger::SetVerbosityLevel(XBot::Logger::Severity::MID);
    
    ros::init(argc, argv, "xbot_cartesian_server");
    ros::NodeHandle nh;
    

    auto model = XBot::ModelInterface::getModel(XBot::Utils::getXBotConfig());
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(model->getUrdf(), kdl_tree);

    robot_state_publisher::RobotStatePublisher rspub(kdl_tree);
    
    tf::TransformBroadcaster tf_broadcaster;

    std::string _urdf_param_name = "/xbotcore/" + model->getUrdf().getName() + "/robot_description";
    std::string _tf_prefix = "/xbotcore/" + model->getUrdf().getName();
    nh.setParam(_urdf_param_name, model->getUrdfString());
    
//     auto obj = (MakeCartesian("wheel_1") + MakeCartesian("wheel_2") + MakeCartesian("wheel_3") + MakeCartesian("wheel_4")) /
//                (MakeCartesian("arm1_7") + MakeCartesian("arm2_7"));
//     ProblemDescription ik_problem = obj;
//     ik_problem << MakeJointLimits() << MakeVelocityLimits();
    
    auto yaml_file = YAML::LoadFile(XBot::Utils::getXBotConfig());
    ProblemDescription ik_problem(yaml_file["CartesianInterface"]["problem_description"]);
    auto sot_ik_solver = std::make_shared<XBot::Cartesian::OpenSotImpl>(model, ik_problem);
    
    XBot::Cartesian::RosServerClass ros_server_class(sot_ik_solver);
    
    ros::Rate loop_rate(100);
    Eigen::VectorXd q, qdot;
    model->getJointPosition(q);
    double time = 0.0;
    double dt = loop_rate.expectedCycleTime().toSec();
    
    while(ros::ok())
    {
        /* Update references from ros */
        ros_server_class.run();
        
        /* Solve ik */
        sot_ik_solver->update(time, dt);
        
        /* Integrate solution */
        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        
        q += dt * qdot;
        
        model->setJointPosition(q);
        model->update();
        
        /* Publish TF */
        XBot::JointNameMap _joint_name_map;
        model->getJointPosition(_joint_name_map);
        std::map<std::string, double> _joint_name_std_map(_joint_name_map.begin(), _joint_name_map.end());

        rspub.publishTransforms(_joint_name_std_map, ros::Time::now(), "");
        
        /* Publish world odom */
        Eigen::Affine3d w_T_pelvis;
        model->getFloatingBasePose(w_T_pelvis);
        tf::Transform transform;
        tf::transformEigenToTF(w_T_pelvis, transform);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(), "pelvis", "world_odom"));
        rspub.publishFixedTransforms("");
        
        /* Update time and sleep */
        time += dt;
        loop_rate.sleep();
    }
    
    return 0;
    
}
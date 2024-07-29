#include <ros/ros.h>
#include <xbot2_interface/ros/config_from_param.hpp>
#include <cartesian_interface/utils/AccMaxComputer.h>

using namespace XBot::Cartesian::Utils;

int main(int argc, char ** argv)
{
    // init ros, get handles
    ros::init(argc, argv, "acc_max_computer_node");
    ros::NodeHandle nh("cartesian");
    ros::NodeHandle nh_priv("~");

    int configs = nh_priv.param("configs", 1e6);
    double initial_qddotmax = nh_priv.param("initial_qddotmax", 1e4);

    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(XBot::Utils::ConfigOptionsFromParamServer());

    ROS_INFO("Initial qddotmax is %f [rad/sec^2]", initial_qddotmax);
    ROS_INFO("Computing max accelerations with %i configurations...", configs);
    std::pair<QDDotMin, QDDotMax> qddot_limits = AccMaxComputer::computeAccMax(model, configs, initial_qddotmax);
    ROS_INFO("...computed!");

    std::vector<std::string> joint_names = model->getJointNames();
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        ROS_INFO_STREAM(joint_names[i]<<" qddot_min: "<<qddot_limits.first[i]<<" qddot_max: "<<qddot_limits.second[i]);



    std::map<std::string,double> map_acc_lims;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
    {
        unsigned int id = model->getDofIndex(joint_names[i]);
        map_acc_lims[joint_names[i]] = std::max(std::fabs(qddot_limits.first[id]), std::fabs(qddot_limits.second[id]));
    }
    nh.setParam("qddot_max", map_acc_lims);

    ROS_INFO("Added ros parameter qddot_max");

    return 0;
}

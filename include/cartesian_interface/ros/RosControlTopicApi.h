#ifndef __XBOT_CARTESIAN_INTERFACE_RosControlTopicApi_H__
#define __XBOT_CARTESIAN_INTERFACE_RosControlTopicApi_H__

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <algorithm>
#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian {

class hw_interface_publisher
{
public:
    hw_interface_publisher(const std::string& interface_name, const std::vector<std::string>& joint_resources, const double dt, ros::NodeHandle& n)
    {
        _interface_name = interface_name;

        _msg.joint_names = joint_resources;
        trajectory_msgs::JointTrajectoryPoint p;
        for(unsigned int i = 0; i < _msg.joint_names.size(); ++i)
        {
            p.positions.push_back(0.);
        }
        p.time_from_start = ros::Duration(dt);
        _msg.points.push_back(p);


        std::string topic_name = "/" + _interface_name + "/command";
        _publisher = n.advertise<trajectory_msgs::JointTrajectory>(topic_name, 1);
    }


    bool has_joint(const std::string& resource)
    {
        if(std::find(_msg.joint_names.begin(), _msg.joint_names.end(), resource) != _msg.joint_names.end())
        {
            return true;
        }
        return false;
    }

    bool set_joint_position(const std::string& resource, const double q)
    {
        auto it = std::find(_msg.joint_names.begin(), _msg.joint_names.end(), resource);
        if(it == _msg.joint_names.end())
            return false;

        int index = it - _msg.joint_names.begin();
        _msg.points[0].positions[index] = q;

        return true;
    }

    const std::string& get_interface_name() const { return _interface_name; }

    const std::vector<std::string>& get_joints() const { return _msg.joint_names; }

    void publish() { _publisher.publish(_msg); }

    private:
        std::string _interface_name;
        ros::Publisher _publisher;
        trajectory_msgs::JointTrajectory _msg;
        bool _is_stopped;

};

class RosControlTopicAPI
{
public:
    typedef std::shared_ptr<RosControlTopicAPI> Ptr;

    /**
     * @brief RosControlTopicAPI
     * @param n ros node handle
     * @param hardware_interface requested hardware interface, e.g. "hardware_interface::PositionJointInterface"
     */
    RosControlTopicAPI(ros::NodeHandle& n, const std::string& hardware_interface, const double dt, ModelInterface::Ptr& model);

    bool startStoppedControllers();

    /**
     * @brief ~RosControlTopicAPI destructor
     */
    virtual ~RosControlTopicAPI(){};

    void setReference(const Eigen::VectorXd& u);
private:
    ros::NodeHandle& _n;
    std::vector<hw_interface_publisher> _hw_interface_publishers;
    ModelInterface::Ptr& _model;



};

}
}

#endif

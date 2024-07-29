#ifndef ROBOTSTATEPUBLISHER_H
#define ROBOTSTATEPUBLISHER_H

#include <xbot2_interface/xbotinterface2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace XBot { namespace Cartesian { namespace Utils {

class RobotStatePublisher
{

public:

    RobotStatePublisher(rclcpp::Node::SharedPtr node,
                        ModelInterface::ConstPtr model);

    void publishTransforms(const rclcpp::Time& time,
                           const std::string& tf_prefix);


private:

    ModelInterface::ConstPtr _model;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    std::vector<geometry_msgs::msg::TransformStamped> _tf_vector;

};

} } }

#endif // ROBOTSTATEPUBLISHER_H

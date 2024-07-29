#include "cartesian_interface/ros/utils/RobotStatePublisher.h"

#include <tf2_eigen/tf2_eigen.hpp>

namespace XBot { namespace Cartesian { namespace Utils {

RobotStatePublisher::RobotStatePublisher(rclcpp::Node::SharedPtr node,
                                         ModelInterface::ConstPtr model):
    _model(model),
    _tf_broadcaster(node)
{

}

void RobotStatePublisher::publishTransforms(const rclcpp::Time & time,
                                            const std::string & tf_prefix)
{
    const auto& urdf = *_model->getUrdf();

    _tf_vector.clear();

    for(const auto& jmap : urdf.joints_)
    {
        const auto& jname = jmap.first;
        const auto& joint = jmap.second;

        const auto& parent_link = joint->parent_link_name;
        const auto& child_link = joint->child_link_name;

        Eigen::Affine3d T;
        _model->getPose(child_link, parent_link, T);

        geometry_msgs::msg::TransformStamped t = tf2::eigenToTransform(T);
        t.header.stamp = time;
        t.header.frame_id = tf_prefix + "/" + parent_link;
        t.child_frame_id = tf_prefix + "/" + child_link;

        _tf_vector.push_back(t);
    }

    _tf_broadcaster.sendTransform(_tf_vector);

}

} } }

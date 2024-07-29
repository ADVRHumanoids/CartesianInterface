#include "cartesian_interface/utils/RobotStatePublisher.h"

#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>

namespace XBot { namespace Cartesian { namespace Utils {

RobotStatePublisher::RobotStatePublisher(ModelInterface::ConstPtr model):
    _model(model)
{

}

void RobotStatePublisher::publishTransforms(const ros::Time & time,
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

        geometry_msgs::TransformStamped tf;
        tf::transformEigenToMsg(T, tf.transform);
        tf.header.stamp = time;
        tf.header.frame_id = tf_prefix + "/" + parent_link;
        tf.child_frame_id = tf_prefix + "/" + child_link;

        _tf_vector.push_back(tf);
    }

    _tf_broadcaster.sendTransform(_tf_vector);

}

} } }

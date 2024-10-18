#ifndef ROBOTSTATEPUBLISHER_H
#define ROBOTSTATEPUBLISHER_H

#include <XBotInterface/ModelInterface.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace XBot { namespace Cartesian { namespace Utils {

class RobotStatePublisher
{

public:

    RobotStatePublisher(ModelInterface::ConstPtr model);

    void publishTransforms(const ros::Time& time,
                           const std::string& tf_prefix);


private:

    ros::Time _last_pub_time;
    ModelInterface::ConstPtr _model;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
    std::vector<geometry_msgs::TransformStamped> _tf_vector;

};

} } }

#endif // ROBOTSTATEPUBLISHER_H

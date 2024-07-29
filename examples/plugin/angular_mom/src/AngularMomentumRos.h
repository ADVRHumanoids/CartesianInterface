#ifndef ANGULARMOMENTUMROS_H
#define ANGULARMOMENTUMROS_H

#include "AngularMomentum.h"
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace XBot { namespace Cartesian {

using geometry_msgs::msg::Vector3Stamped;

namespace ServerApi
{
class AngularMomentumRos;
}

/**
 * @brief The ServerApi::AngularMomentumRos class implements a ROS
 * interface for the task.
 */
class ServerApi::AngularMomentumRos : public ServerApi::TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(AngularMomentumRos)

    AngularMomentumRos(TaskDescription::Ptr task,
                       RosContext::Ptr ros_context);

    void run(rclcpp::Time time) override;


private:

    void on_ref_recv(Vector3Stamped::ConstSharedPtr msg);

    rclcpp::Subscription<Vector3Stamped>::SharedPtr _ref_sub;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr _cur_ref_pub;

    AngularMomentum::Ptr _ci_angmom;


};

} }

#endif // ANGULARMOMENTUMROS_H

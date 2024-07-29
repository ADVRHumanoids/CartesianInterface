#ifndef POSTURALROS_SERVERAPI_H
#define POSTURALROS_SERVERAPI_H

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

#include <cartesian_interface/problem/Postural.h>

#include <sensor_msgs/msg/joint_state.hpp>

#include <rclcpp/rclcpp.hpp>

namespace XBot { namespace Cartesian {

using sensor_msgs::msg::JointState;

namespace ServerApi
{
    class PosturalRos;
}

class ServerApi::PosturalRos : public ServerApi::TaskRos
{

public:

    PosturalRos(PosturalTask::Ptr task,
                RosContext::Ptr context);

    virtual void run(rclcpp::Time time) override;

protected:

private:

    void on_ref_recv(JointState::ConstSharedPtr msg);

    PosturalTask::Ptr _postural;

    rclcpp::Publisher<JointState>::SharedPtr _current_ref_pub;
    rclcpp::SubscriptionBase::SharedPtr _ref_sub;

    Eigen::VectorXd _posture_ref;





};

} }

#endif // POSTURALROS_H

#ifndef POSTURALROS__CLIENTAPI_H
#define POSTURALROS__CLIENTAPI_H

#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
class PosturalRos;
}

class ClientApi::PosturalRos : virtual public PosturalTask,
        public ClientApi::TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(PosturalRos)

    PosturalRos(std::string name,
                rclcpp::Node::SharedPtr node);

    bool validate() override;

    bool useInertiaMatrixWeight() const override;

    void getReferencePosture(Eigen::VectorXd& qref) const override;

    void getReferencePosture(JointNameMap& qref) const override;

    void setReferencePosture(const JointNameMap& qref) override;

    void setReferencePosture(const Eigen::VectorXd& qref) override;

    void setReferenceVelocity(const JointNameMap& qdotref) override;

    void setReferenceVelocity(const Eigen::VectorXd& qdotref) override;

    void getReferenceVelocity(Eigen::VectorXd& qdotref) const override;

private:

    void on_current_ref_recv(sensor_msgs::msg::JointState::ConstSharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  _ref_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _current_ref_sub;

    JointNameMap _current_ref;

    bool _curr_ref_recv;

};

} }

#endif // POSTURALROS_H

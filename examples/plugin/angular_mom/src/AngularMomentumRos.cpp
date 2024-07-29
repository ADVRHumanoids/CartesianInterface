#include "AngularMomentumRos.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include "fmt/format.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;

AngularMomentumRos::AngularMomentumRos(TaskDescription::Ptr task,
                                       RosContext::Ptr ros_context):
    TaskRos(task, ros_context)
{
    /* Type cast to the required type, and throw on failure */
    _ci_angmom = std::dynamic_pointer_cast<AngularMomentum>(task);
    if(!_ci_angmom) throw std::runtime_error("Provided task description "
                                             "does not have expected type 'AngularMomentum'");

    /* Open topics */
    _ref_sub = _ctx->node()->create_subscription<Vector3Stamped>(
        task->getName() + "/reference", 1,
        std::bind(&AngularMomentumRos::on_ref_recv, this, std::placeholders::_1)
        );

    _cur_ref_pub = _ctx->node()->create_publisher<Vector3Stamped>(task->getName() + "/current_reference",
                                                                  1);
    /* Register type name */
    registerType("AngularMomentum");
}

void AngularMomentumRos::run(rclcpp::Time time)
{
    Vector3Stamped msg;
    msg.header.stamp = time;

    tf2::toMsg(_ci_angmom->getReference(), msg.vector);

    _cur_ref_pub->publish(msg);
}

void AngularMomentumRos::on_ref_recv(Vector3Stamped::ConstSharedPtr msg)
{
    Eigen::Vector3d ref;
    tf2::fromMsg(msg->vector, ref);

    _ci_angmom->setReference(ref);
}

CARTESIO_REGISTER_ROS_API_PLUGIN(AngularMomentumRos, AngularMomentum)

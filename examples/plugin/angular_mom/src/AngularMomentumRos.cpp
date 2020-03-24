#include "AngularMomentumRos.h"
#include <eigen_conversions/eigen_msg.h>
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
    _ref_sub = _ctx->nh().subscribe(task->getName() + "/reference", 1,
                                   &AngularMomentumRos::on_ref_recv, this);

    _cur_ref_pub = _ctx->nh().advertise<geometry_msgs::Vector3Stamped>(task->getName() + "/current_reference",
                                                                  1);
    /* Register type name */
    registerType("AngularMomentum");
}

void AngularMomentumRos::run(ros::Time time)
{
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = time;

    tf::vectorEigenToMsg(_ci_angmom->getReference(),
                         msg.vector);

    _cur_ref_pub.publish(msg);
}

void AngularMomentumRos::on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg)
{
    Eigen::Vector3d ref;
    tf::vectorMsgToEigen(msg->vector, ref);

    _ci_angmom->setReference(ref);
}

CARTESIO_REGISTER_ROS_API_PLUGIN(AngularMomentumRos, AngularMomentum)

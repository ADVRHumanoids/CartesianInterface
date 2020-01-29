#include "InteractionRos.h"
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;


InteractionRos::InteractionRos(InteractionTask::Ptr task,
                               XBot::ModelInterface::ConstPtr model):
    CartesianRos(task, model)
{
    _fref_pub = _ctx.nh().advertise<geometry_msgs::WrenchStamped>(task->getName() + "/current_reference", 1);
    _fref_sub = _ctx.nh().subscribe(task->getName() + "/reference", 1,
                                    &InteractionRos::on_fref_recv, this);

    registerType("Interaction");
}

void InteractionRos::run(ros::Time time)
{
    geometry_msgs::WrenchStamped msg;
    tf::wrenchEigenToMsg(_ci_inter->getForceReference(), msg.wrench);

    _fref_pub.publish(msg);
}

void InteractionRos::on_fref_recv(geometry_msgs::WrenchStampedConstPtr msg)
{
    Eigen::Vector6d fref;
    tf::wrenchMsgToEigen(msg->wrench, fref);

    _ci_inter->setForceReference(fref);
}

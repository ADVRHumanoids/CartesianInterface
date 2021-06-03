#include "ros/client_api/PosturalRos.h"
#include "fmt/format.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;


PosturalRos::PosturalRos(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh),
    _curr_ref_recv(false)
{
    _ref_pub = _nh.advertise<sensor_msgs::JointState>(name + "/reference", 5);

    _current_ref_sub = _nh.subscribe(name + "/current_reference", 1,
                                     &PosturalRos::on_current_ref_recv, this);
}

bool PosturalRos::validate()
{
    return TaskRos::validate() && _curr_ref_recv;
}

bool PosturalRos::useInertiaMatrixWeight() const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

void PosturalRos::getReferencePosture(Eigen::VectorXd & qref) const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

void PosturalRos::getReferencePosture(XBot::JointNameMap & qref) const
{
    qref = _current_ref;
}

void PosturalRos::setReferencePosture(const XBot::JointNameMap & qref)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.name.reserve(qref.size());
    msg.position.reserve(qref.size());

    for(const auto& pair : qref)
    {
        msg.name.push_back(pair.first);
        msg.position.push_back(pair.second);
    }

    _ref_pub.publish(msg);
}

void PosturalRos::setReferencePosture(const Eigen::VectorXd & qref)
{
    ///TODO
}

void PosturalRos::on_current_ref_recv(sensor_msgs::JointStateConstPtr msg)
{
    _curr_ref_recv = true;

    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->position.size() <= i)
        {
            continue;
        }

        _current_ref[msg->name[i]] = msg->position[i];
    }

}

void PosturalRos::setReferenceVelocity(const JointNameMap& qdotref)
{
    ///TODO
}

void PosturalRos::getReferenceVelocity(Eigen::VectorXd& qdotref) const
{
    ///TODO
}

void PosturalRos::setReferenceVelocity(const Eigen::VectorXd& qdotref)
{
    ///TODO
}

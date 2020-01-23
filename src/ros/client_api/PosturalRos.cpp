#include "PosturalRos.h"
#include "fmt/format.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;


PosturalRos::PosturalRos(std::string name, ros::NodeHandle nh):
    TaskRos(name, nh)
{
    _ref_pub = _nh.advertise<sensor_msgs::JointState>(name + "/reference", 5);

    _current_ref_sub = _nh.subscribe(name + "/current_reference", 1,
                                     &PosturalRos::on_current_ref_recv, this);
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

void PosturalRos::on_current_ref_recv(sensor_msgs::JointStateConstPtr msg)
{

    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->position.size() <= i)
        {
            continue;
        }

        _current_ref[msg->name[i]] = msg->position[i];
    }

}
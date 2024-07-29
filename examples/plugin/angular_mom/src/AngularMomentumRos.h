#ifndef ANGULARMOMENTUMROS_H
#define ANGULARMOMENTUMROS_H

#include "AngularMomentum.h"
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace XBot { namespace Cartesian {

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


    AngularMomentumRos(TaskDescription::Ptr task,
                       RosContext::Ptr ros_context);

    void run(ros::Time time) override;


private:

    void on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg);

    ros::Subscriber _ref_sub;
    ros::Publisher _cur_ref_pub;

    AngularMomentum::Ptr _ci_angmom;


};


namespace ClientApi
{
class AngularMomentumRos;
}

class ClientApi::AngularMomentumRos : virtual public AngularMomentum, public ClientApi::TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(AngularMomentumRos)

    AngularMomentumRos(std::string name,
                ros::NodeHandle nh):TaskRos(name, nh) {}



private:


};

}}

#endif // ANGULARMOMENTUMROS_H

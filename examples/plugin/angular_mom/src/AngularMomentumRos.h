#ifndef ANGULARMOMENTUMROS_H
#define ANGULARMOMENTUMROS_H

#include "AngularMomentum.h"
#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>
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

    CARTESIO_DECLARE_SMART_PTR(AngularMomentumRos)

    AngularMomentumRos(TaskDescription::Ptr task,
                       ModelInterface::ConstPtr model);

    void run(ros::Time time) override;


private:

    void on_ref_recv(geometry_msgs::Vector3StampedConstPtr msg);

    ros::Subscriber _ref_sub;
    ros::Publisher _cur_ref_pub;

    AngularMomentum::Ptr _ci_angmom;


};

} }

#endif // ANGULARMOMENTUMROS_H

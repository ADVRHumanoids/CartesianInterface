#ifndef INTERACTIONROS_H
#define INTERACTIONROS_H

#include <geometry_msgs/WrenchStamped.h>

#include "CartesianRos.h"
#include <cartesian_interface/problem/Interaction.h>

namespace XBot { namespace Cartesian {


namespace ServerApi
{
class InteractionRos;
}

class ServerApi::InteractionRos : public ServerApi::CartesianRos
{

public:

    InteractionRos(InteractionTask::Ptr task,
                   ModelInterface::ConstPtr model);

    virtual void run(ros::Time time) override;

private:

    void on_fref_recv(geometry_msgs::WrenchStampedConstPtr msg);

    ros::Subscriber _fref_sub;
    ros::Publisher _fref_pub;

    InteractionTask::Ptr _ci_inter;

};

} }


#endif // INTERACTIONROS_H

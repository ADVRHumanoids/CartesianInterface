#ifndef POSTURALROS_SERVERAPI_H
#define POSTURALROS_SERVERAPI_H

#include <cartesian_interface/sdk/ros/server_api/TaskRos.h>

#include <cartesian_interface/problem/Postural.h>
#include <sensor_msgs/JointState.h>

namespace XBot { namespace Cartesian {

namespace ServerApi
{
    class PosturalRos;
}

class ServerApi::PosturalRos : public ServerApi::TaskRos
{

public:

    PosturalRos(PosturalTask::Ptr task,
                 ModelInterface::ConstPtr model);

    virtual void run(ros::Time time) override;

protected:

private:

    void on_ref_recv(sensor_msgs::JointStateConstPtr msg);

    PosturalTask::Ptr _postural;

    ros::Publisher _current_ref_pub;
    ros::Subscriber _ref_sub;

    Eigen::VectorXd _posture_ref;





};

} }

#endif // POSTURALROS_H

#ifndef POSTURALROS_H
#define POSTURALROS_H

#include <cartesian_interface/problem/Postural.h>
#include "TaskRos.h"

#include <sensor_msgs/JointState.h>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
class PosturalRos;
}

class ClientApi::PosturalRos : virtual public PosturalTask,
        public ClientApi::TaskRos
{

public:

    PosturalRos(std::string name,
                ros::NodeHandle nh);

    bool validate() override;

    bool useInertiaMatrixWeight() const override;

    void getReferencePosture(Eigen::VectorXd& qref) const override;

    void getReferencePosture(JointNameMap& qref) const override;

    void setReferencePosture(const JointNameMap& qref) override;

private:

    void on_current_ref_recv(sensor_msgs::JointStateConstPtr msg);

    ros::Publisher  _ref_pub;
    bool _curr_ref_recv;
    ros::Subscriber _current_ref_sub;

    JointNameMap _current_ref;

};

} }

#endif // POSTURALROS_H

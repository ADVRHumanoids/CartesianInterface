#ifndef POSTURALROS__CLIENTAPI_H
#define POSTURALROS__CLIENTAPI_H

#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/sdk/ros/client_api/TaskRos.h>

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

    CARTESIO_DECLARE_SMART_PTR(PosturalRos)

    PosturalRos(std::string name,
                ros::NodeHandle nh);

    bool validate() override;

    bool useInertiaMatrixWeight() const override;

    void getReferencePosture(Eigen::VectorXd& qref) const override;

    void getReferencePosture(JointNameMap& qref) const override;

    void setReferencePosture(const JointNameMap& qref) override;

    void setReferencePosture(const Eigen::VectorXd& qref) override;

    void setReferenceVelocity(const JointNameMap& qdotref) override;

    void setReferenceVelocity(const Eigen::VectorXd& qdotref) override;

    void getReferenceVelocity(Eigen::VectorXd& qdotref) const override;

private:

    void on_current_ref_recv(sensor_msgs::JointStateConstPtr msg);

    ros::Publisher  _ref_pub;
    bool _curr_ref_recv;
    ros::Subscriber _current_ref_sub;

    JointNameMap _current_ref;

};

} }

#endif // POSTURALROS_H

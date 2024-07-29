#include "ros/server_api/PosturalRos.h"
#include "../utils/RosUtils.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;

PosturalRos::PosturalRos(PosturalTask::Ptr task,
                         RosContext::Ptr context):
    TaskRos(task, context),
    _postural(task)
{
    registerType("Postural");

    _current_ref_pub = context->node()->create_publisher<JointState>(
        task->getName() + "/current_reference", 1);

    _ref_sub = ::create_subscription<JointState>(context->node(),
                                                 task->getName() + "/reference",
                                                 &PosturalRos::on_ref_recv,
                                                 this,
                                                 1);
}

void PosturalRos::run(rclcpp::Time time)
{
    TaskRos::run(time);

    JointState msg;

    if(_current_ref_pub->get_subscription_count() == 0)
    {
        return;
    }

    // get posture reference (size = nq)
    _postural->getReferencePosture(_posture_ref);

    // get minimal representation of q
    // this has size = nv
    _model->positionToMinimal(_posture_ref, _posture_ref);

    // to deal with non-euclidean joints, we will publish the
    // minimal representation of q

    msg.header.stamp = time;
    msg.name.reserve(_model->getNv());
    msg.position.reserve(_model->getNv());
    int i = 0;
    for(const std::string& jname : _model->getVNames())
    {
        msg.name.push_back(jname);
        msg.position.push_back(_posture_ref[i]);
        i++;
    }

    _current_ref_pub->publish(msg);
}

void PosturalRos::on_ref_recv(JointState::ConstSharedPtr msg)
{
    JointNameMap posture_ref;

    for(int i = 0; i < msg->name.size(); i++)
    {
        if(msg->position.size() <= i)
        {
            continue;
        }

        posture_ref[msg->name[i]] = msg->position[i];
    }

    _postural->setReferencePosture(posture_ref);
}

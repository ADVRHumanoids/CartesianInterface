#include "ros/server_api/PosturalRos.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;

PosturalRos::PosturalRos(PosturalTask::Ptr task,
                         RosContext::Ptr context):
    TaskRos(task, context),
    _postural(task)
{
    registerType("Postural");

    _current_ref_pub = _ctx->nh().advertise<sensor_msgs::JointState>(task->getName() + "/current_reference", 1);

    _ref_sub = _ctx->nh().subscribe(task->getName() + "/reference", 1,
                                   &PosturalRos::on_ref_recv, this);
}

void PosturalRos::run(ros::Time time)
{
    TaskRos::run(time);

    sensor_msgs::JointState msg;

    if(_current_ref_pub.getNumSubscribers() == 0)
    {
        return;
    }

    // get posture reference (size = nq)
    _postural->getReferencePosture(_posture_ref);

    // apply log map to retrieve the motion representation of q
    // this has size = nv
    _posture_ref = _model->difference(_posture_ref, _model->getNeutralQ());

    // to deal with non-euclidean joints, we will publish the
    // log map of q, i.e. the motion that brings the robot to q
    // when applied for unit time starting from q0

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

    _current_ref_pub.publish(msg);
}

void PosturalRos::on_ref_recv(sensor_msgs::JointStateConstPtr msg)
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

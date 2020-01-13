#include "TaskRos.h"

using namespace XBot::Cartesian;

TaskRos::TaskRos(RosContext ctx, TaskDescription::Ptr task):
    _ctx(ctx),
    _task(task),
    task_name(task->getName())
{
    _lambda_pub = _ctx.nh().advertise<std_msgs::Float32>(task_name + "/lambda", 1);
    _lambda_sub = _ctx.nh().subscribe(task_name + "/lambda_desired", 1,
                                &TaskRos::on_lambda_recv, this);
}

void XBot::Cartesian::TaskRos::run(ros::Time time)
{
    publish_lambda();
}

void TaskRos::publish_lambda()
{
    std_msgs::Float32 msg;

    msg.data = _task->getLambda();

    _lambda_pub.publish(msg);
}

void TaskRos::on_lambda_recv(std_msgs::Float32ConstPtr msg)
{
    _task->setLambda(msg->data);
}

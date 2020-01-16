#include "TaskRos.h"
#include "utils/DynamicLoading.h"
#include "fmt/format.h"

#include "CartesianRos.h"

using namespace XBot::Cartesian;

TaskRos::TaskRos(TaskDescription::Ptr task):
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

RosApiNotFound::RosApiNotFound(std::string message): _msg(message)
{}

const char * RosApiNotFound::what() const noexcept
{
    return _msg.c_str();
}


TaskRos::Ptr TaskRos::MakeInstance(TaskDescription::Ptr task)
{
    TaskRos * ros_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(task->getType() == "Cartesian") /* Otherwise, construct supported tasks */
    {
        ros_adapter = new CartesianRos(task);
    }
    //    else if(task->getType() == "Postural")
    //    {
    //    }
    //    else if(task->getType() == "Com")
    //    {
    //    }
    else
    {
        try
        {
            ros_adapter = CallFunction<TaskRos*>(RosApiPluginName(task),
                                                 "create_cartesio_ros_api",
                                                 task);
        }
        catch(LibNotFound&)
        {
            auto str = fmt::format("Unable to construct TaskRos instance for task '{}': "
                                   "lib '{}' not found for unsupported task type '{}'",
                                   task->getName(), RosApiPluginName(task), task->getType());

            throw RosApiNotFound(str);
        }
    }

    Ptr rosapi_shared_ptr(ros_adapter);

    return rosapi_shared_ptr;
}

std::string TaskRos::RosApiPluginName(TaskDescription::Ptr task)
{
    return "libcartesio_ros_api_" + task->getType() + ".so";
}

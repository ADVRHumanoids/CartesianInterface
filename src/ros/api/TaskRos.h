#ifndef TASKROS_H
#define TASKROS_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ros/RosContext.h>

namespace XBot { namespace Cartesian {

class TaskRos
{

public:

    CARTESIO_DECLARE_SMART_PTR(TaskRos)

    explicit TaskRos(TaskDescription::Ptr task);

    virtual void run(ros::Time time);

    virtual ~TaskRos() = default;

    static Ptr MakeInstance(TaskDescription::Ptr task);

protected:

    const std::string task_name;

    RosContext _ctx;
    TaskDescription::Ptr _task;

private:

    static std::string RosApiPluginName(TaskDescription::Ptr task);

    void publish_lambda();

    void on_lambda_recv(std_msgs::Float32ConstPtr msg);

    ros::Publisher _lambda_pub;
    ros::Subscriber _lambda_sub;



};

struct RosApiNotFound : public std::exception
{
    RosApiNotFound(std::string message);

    const char * what() const noexcept;

    std::string _msg;
};

} }

#endif // TASKROS_H

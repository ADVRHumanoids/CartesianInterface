#ifndef TASKROS_H
#define TASKROS_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace XBot { namespace Cartesian {

class RosContext
{

public:

    ros::NodeHandle& nh();
    const std::string& tf_prefix() const;
    const std::string& tf_prefix_slash() const;

};

class TaskRos
{

public:

    TaskRos(RosContext ctx,
            TaskDescription::Ptr task);

    virtual void run(ros::Time time);

    virtual ~TaskRos() = default;

protected:

    const std::string task_name;

    RosContext _ctx;
    TaskDescription::Ptr _task;

private:

    void publish_lambda();

    void on_lambda_recv(std_msgs::Float32ConstPtr msg);

    ros::Publisher _lambda_pub;
    ros::Subscriber _lambda_sub;



};

} }

#endif // TASKROS_H

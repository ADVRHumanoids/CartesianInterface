#ifndef TASKROS_H
#define TASKROS_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetWeight.h>
#include <cartesian_interface/SetLambda.h>
#include <cartesian_interface/SetTaskActive.h>
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
    XBot::ModelInterface::ConstPtr _model;

private:

    static std::string RosApiPluginName(TaskDescription::Ptr task);


    bool get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                          cartesian_interface::GetTaskInfoResponse& res);

    bool set_lambda_cb(cartesian_interface::SetLambdaRequest& req,
                       cartesian_interface::SetLambdaResponse& res);

    bool set_weight_cb(cartesian_interface::SetWeightRequest& req,
                       cartesian_interface::SetWeightResponse& res);

    bool set_active_cb(cartesian_interface::SetTaskActiveRequest& req,
                       cartesian_interface::SetTaskActiveResponse& res);



    ros::ServiceServer _get_task_info_srv, _set_lambda_srv, _set_weight_srv;
    ros::ServiceServer _set_active_srv;



};

struct RosApiNotFound : public std::exception
{
    RosApiNotFound(std::string message);

    const char * what() const noexcept;

    std::string _msg;
};

} }

#endif // TASKROS_H

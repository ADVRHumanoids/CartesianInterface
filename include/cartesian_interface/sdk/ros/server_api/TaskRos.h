#ifndef TASKROS_SERVERAPI_H
#define TASKROS_SERVERAPI_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetWeight.h>
#include <cartesian_interface/SetLambda.h>
#include <cartesian_interface/SetLambda2.h>
#include <cartesian_interface/SetTaskActive.h>
#include <cartesian_interface/sdk/ros/RosContext.h>
#include <cartesian_interface/sdk/ros/Plugin.h>

namespace XBot { namespace Cartesian {

namespace ServerApi
{
    class TaskRos;
}

class ServerApi::TaskRos : public virtual TaskObserver
{

public:

    CARTESIO_DECLARE_SMART_PTR(TaskRos)

    TaskRos(TaskDescription::Ptr task,
            RosContext::Ptr context);

    virtual void run(ros::Time time);

    bool onActivationStateChanged() override;

    static Ptr MakeInstance(TaskDescription::Ptr task,
                            RosContext::Ptr context);

protected:

    virtual bool initialize();

    void notifyTaskChanged(const std::string& msg);

    void registerType(const std::string& type);

    const std::string task_name;

    RosContext::Ptr _ctx;
    TaskDescription::Ptr _task;
    ModelInterface::ConstPtr _model;

private:

    static std::string RosApiPluginName(TaskDescription::Ptr task);


    bool get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                          cartesian_interface::GetTaskInfoResponse& res);

    bool set_lambda_cb(cartesian_interface::SetLambdaRequest& req,
                       cartesian_interface::SetLambdaResponse& res);

    bool set_lambda2_cb(cartesian_interface::SetLambda2Request& req,
                        cartesian_interface::SetLambda2Response& res);

    bool set_weight_cb(cartesian_interface::SetWeightRequest& req,
                       cartesian_interface::SetWeightResponse& res);

    bool set_active_cb(cartesian_interface::SetTaskActiveRequest& req,
                       cartesian_interface::SetTaskActiveResponse& res);


    std::list<std::string> _type_hierarchy;

    ros::ServiceServer _get_task_info_srv, _set_lambda_srv, _set_lambda2_srv, _set_weight_srv;
    ros::ServiceServer _set_active_srv;

    ros::Publisher _task_changed_pub;


};

struct RosApiNotFound : public std::exception
{
    RosApiNotFound(std::string message);

    const char * what() const noexcept;

    std::string _msg;
};

} }

#endif // TASKROS_H

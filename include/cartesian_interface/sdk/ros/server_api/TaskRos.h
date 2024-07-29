#ifndef TASKROS_SERVERAPI_H
#define TASKROS_SERVERAPI_H

#include <cartesian_interface/problem/Task.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cartesian_interface/srv/get_task_info.hpp>
#include <cartesian_interface/srv/set_weight.hpp>
#include <cartesian_interface/srv/set_lambda.hpp>
#include <cartesian_interface/srv/set_lambda2.hpp>
#include <cartesian_interface/srv/set_task_active.hpp>
#include <cartesian_interface/msg/task_info.hpp>

#include <cartesian_interface/sdk/ros/RosContext.h>
#include <cartesian_interface/sdk/ros/Plugin.h>

namespace XBot { namespace Cartesian {

using namespace cartesian_interface::msg;
using namespace cartesian_interface::srv;

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

    virtual void run(rclcpp::Time time);

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


    bool get_task_info_cb(GetTaskInfo::Request::ConstSharedPtr req,
                          GetTaskInfo::Response::SharedPtr res);

    bool set_lambda_cb(SetLambda::Request::ConstSharedPtr req,
                       SetLambda::Response::SharedPtr res);

    bool set_lambda2_cb(SetLambda2::Request::ConstSharedPtr req,
                        SetLambda2::Response::SharedPtr res);

    bool set_weight_cb(SetWeight::Request::ConstSharedPtr req,
                       SetWeight::Response::SharedPtr res);

    bool set_active_cb(SetTaskActive::Request::ConstSharedPtr req,
                       SetTaskActive::Response::SharedPtr res);


    std::list<std::string> _type_hierarchy;

    std::vector<rclcpp::ServiceBase::SharedPtr> _srv;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _task_changed_pub;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _task_err_pub;

    rclcpp::Publisher<TaskInfo>::SharedPtr _task_info_pub;

    Eigen::VectorXd _err;

};

struct RosApiNotFound : public std::exception
{
    RosApiNotFound(std::string message);

    const char * what() const noexcept;

    std::string _msg;
};

} }

#endif // TASKROS_H

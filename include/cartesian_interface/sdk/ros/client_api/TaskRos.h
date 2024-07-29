#ifndef TASKROS_CLIENTAPI_H
#define TASKROS_CLIENTAPI_H

#include <cartesian_interface/problem/Task.h>
#include <rclcpp/rclcpp.hpp>

#include <cartesian_interface/srv/get_task_info.hpp>
#include <cartesian_interface/msg/task_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <cartesian_interface/srv/set_task_active.hpp>
#include <cartesian_interface/srv/set_lambda.hpp>
#include <cartesian_interface/srv/set_lambda2.hpp>
#include <cartesian_interface/srv/set_weight.hpp>
#include <cartesian_interface/srv/set_task_active.hpp>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
    class TaskRos;
}

using cartesian_interface::msg::TaskInfo;
using cartesian_interface::srv::GetTaskInfo;
using cartesian_interface::srv::SetWeight;
using cartesian_interface::srv::SetLambda;
using cartesian_interface::srv::SetLambda2;
using cartesian_interface::srv::SetTaskActive;

class ClientApi::TaskRos : virtual public TaskDescription
{

public:

    TaskRos(std::string name,
            rclcpp::Node::SharedPtr node);

    bool validate() override;

    void update(double time, double period) override;

    void reset() override;

    void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;

    const Eigen::MatrixXd & getWeight() const override;

    bool setWeight(const Eigen::MatrixXd & value) override;

    const std::vector<int> & getIndices() const override;

    void setIndices(const std::vector<int> & value) override;

    double getLambda() const override;

    void setLambda(double value) override;

    double getLambda2() const override;

    bool setLambda2(double value) override;

    const std::vector<std::string> & getDisabledJoints() const override;

    void setDisabledJoints(const std::vector<std::string> & value) override;

    ActivationState getActivationState() const override;

    bool setActivationState(const ActivationState & value) override;

    const std::string & getName() const override;

    const std::string & getType() const override;

    int getSize() const override;

    const std::string & getLibName() const override;

    void registerObserver(TaskObserver::WeakPtr obs) override;

    void setAsyncMode(bool is_async);
    bool asyncMode() const;

    static Ptr MakeInstance(std::string name,
                            std::vector<std::string> type,
                            std::string lib_name,
                            rclcpp::Node::SharedPtr node);

protected:

    rclcpp::Node::SharedPtr _node;

    virtual void notifyTaskChanged(const std::string& message);

private:

    GetTaskInfo::Response get_task_info() const;

    void on_task_info_recv(cartesian_interface::msg::TaskInfo::ConstSharedPtr msg);
    void on_task_changed_ev_recv(std_msgs::msg::String::ConstSharedPtr msg);

    bool _async_update;

    std::string _name;
    mutable std::string _type;
    double _lambda;
    ActivationState _activ_state;
    int _size;

    mutable std::vector<std::string> _disabled_joints;
    mutable std::vector<int> _indices;
    mutable Eigen::MatrixXd _weight;

    std::list<TaskObserver::WeakPtr> _observers;

    rclcpp::Client<GetTaskInfo>::SharedPtr _task_prop_cli;
    rclcpp::Client<SetWeight>::SharedPtr _set_weight_cli;
    rclcpp::Client<SetLambda>::SharedPtr _set_lambda_cli;
    rclcpp::Client<SetLambda2>::SharedPtr _set_lambda2_cli;
    rclcpp::Client<SetTaskActive>::SharedPtr _activate_cli;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _task_changed_sub;
    rclcpp::Subscription<TaskInfo>::SharedPtr _task_info_sub;

    cartesian_interface::msg::TaskInfo _info;

    bool _async;

};

} }
#endif // TASKROS_H

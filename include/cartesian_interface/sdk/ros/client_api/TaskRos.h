#ifndef TASKROS_CLIENTAPI_H
#define TASKROS_CLIENTAPI_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>

#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/TaskInfo.h>
#include <std_msgs/String.h>

namespace XBot { namespace Cartesian {

namespace ClientApi
{
    class TaskRos;
}

class ClientApi::TaskRos : virtual public TaskDescription
{

public:

    TaskRos(std::string name,
            ros::NodeHandle nh);

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
                            ros::NodeHandle nh);

protected:

    ros::NodeHandle _nh;

    virtual void notifyTaskChanged(const std::string& message);

private:

    cartesian_interface::GetTaskInfoResponse get_task_info() const;

    void on_task_info_recv(cartesian_interface::TaskInfoConstPtr msg);
    void on_task_changed_ev_recv(std_msgs::StringConstPtr msg);

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

    mutable ros::ServiceClient _task_prop_cli;
    ros::ServiceClient _set_weight_cli;
    ros::ServiceClient _set_lambda_cli, _set_lambda2_cli;
    ros::ServiceClient _activate_cli;
    ros::Subscriber _task_changed_sub;
    ros::Subscriber _task_info_sub;

    cartesian_interface::TaskInfo _info;

    bool _async;

};

} }
#endif // TASKROS_H

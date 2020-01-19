#ifndef TASKROS_H
#define TASKROS_H

#include <cartesian_interface/problem/Task.h>
#include <ros/ros.h>

namespace XBot { namespace Cartesian {

class TaskRos : virtual public TaskDescription
{

public:

    TaskRos(std::string name);

    bool validate() override;

    void update(double time, double period) override;

    void reset() override;

    void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;

    const Eigen::MatrixXd & getWeight() const override;

    bool setWeight(const Eigen::MatrixXd & value) override;

    const std::vector<int> & getIndices() const override;

    void setIndices(const std::vector<int> & value) override;

    double getLambda() const override;

    void setLambda(double value) override;

    const std::vector<std::string> & getDisabledJoints() const override;

    void setDisabledJoints(const std::vector<std::string> & value) override;

    ActivationState getActivationState() const override;

    bool setActivationState(const ActivationState & value) override;

    virtual ~TaskRos();

protected:

    ros::NodeHandle _nh;

private:

    ros::ServiceClient _task_prop_cli;
    ros::ServiceClient _set_weight_cli;
    ros::ServiceClient _set_lambda_cli;
    ros::ServiceClient _activate_cli;
};

} }
#endif // TASKROS_H

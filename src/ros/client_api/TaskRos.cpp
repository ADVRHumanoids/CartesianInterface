#include "TaskRos.h"
#include "fmt/format.h"

#include <cartesian_interface/SetLambda.h>
#include <cartesian_interface/SetWeight.h>
#include <cartesian_interface/SetTaskActive.h>

#include "CartesianRos.h"
#include "PosturalRos.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;

TaskRos::TaskRos(std::string name,
                 ros::NodeHandle nh):
    _nh(nh),
    _name(name),
    _async(false)
{
    _task_prop_cli = _nh.serviceClient<cartesian_interface::GetTaskInfo>(name + "/get_task_properties");
    if(!_task_prop_cli.waitForExistence(ros::Duration(1.0)) || !_task_prop_cli.exists())
    {
        throw std::runtime_error(fmt::format("Non existent service '{}'",
                                             _task_prop_cli.getService()));
    }

    _set_lambda_cli = _nh.serviceClient<cartesian_interface::SetLambda>(name + "/set_lambda");
    _set_weight_cli = _nh.serviceClient<cartesian_interface::SetWeight>(name + "/set_weight");
    _activate_cli = _nh.serviceClient<cartesian_interface::SetTaskActive>(name + "/set_active");

}

bool TaskRos::validate()
{
    return true;
}

void TaskRos::update(double time, double period)
{

}

void TaskRos::reset()
{

}

void TaskRos::log(XBot::MatLogger::Ptr logger, bool init_logger, int buf_size)
{

}


const Eigen::MatrixXd & TaskRos::getWeight() const
{
    int size = getSize();

    auto res = get_task_info();
    _weight = Eigen::MatrixXf::Map(res.weight.data(),
                                   size, size).cast<double>();
    return _weight;
}

bool TaskRos::setWeight(const Eigen::MatrixXd & value)
{
    int size = getSize();

    if(value.rows() != size || value.cols() != size)
    {

        Logger::error("%s \n",
                      fmt::format("Invalid weight size ({} x {}), expected {} x {}",
                                  value.rows(), value.cols(),
                                  size, size).c_str());

        return false;
    }

    cartesian_interface::SetWeight srv;
    srv.request.weight.resize(size*size);
    Eigen::MatrixXf::Map(srv.request.weight.data(),
                         size, size) = value.cast<float>();

    if(!_set_weight_cli.call(srv))
    {
        return false;
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;

}

const std::vector<int>& TaskRos::getIndices() const
{
    auto res = get_task_info();

    _indices.assign(res.indices.begin(),
                    res.indices.end());

    return _indices;
}

void TaskRos::setIndices(const std::vector<int> & value)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

double TaskRos::getLambda() const
{
    auto res = get_task_info();

    return res.lambda;
}

void TaskRos::setLambda(double value)
{
    cartesian_interface::SetLambda srv;
    srv.request.lambda = value;

    if(!_set_lambda_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _set_lambda_cli.getService()));
    }

    if(!srv.response.success)
    {
        throw std::runtime_error(fmt::format("Service '{}' returned false: {}",
                                             _set_lambda_cli.getService(),
                                             srv.response.message));
    }

    ROS_INFO("%s", srv.response.message.c_str());

}

const std::vector<std::string> & TaskRos::getDisabledJoints() const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

void TaskRos::setDisabledJoints(const std::vector<std::string> & value)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

ActivationState TaskRos::getActivationState() const
{
    auto res = get_task_info();

    if(res.activation_state == "Enabled") return ActivationState::Enabled;
    else return ActivationState::Disabled;
}

bool TaskRos::setActivationState(const ActivationState & value)
{
    cartesian_interface::SetTaskActive srv;
    srv.request.activation_state = (value == ActivationState::Enabled);

    if(!_activate_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _activate_cli.getService()));
    }

    ROS_INFO("%s", srv.response.message.c_str());

    return srv.response.success;
}


const std::string & TaskRos::getName() const
{
    return _name;
}

const std::string & TaskRos::getType() const
{
    _type = get_task_info().type;
    return _type;
}

int TaskRos::getSize() const
{
    return get_task_info().size;
}

const std::string & TaskRos::getLibName() const
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

void TaskRos::registerObserver(TaskObserver::WeakPtr obs)
{
}

void TaskRos::setAsyncMode(bool is_async)
{
    _async = is_async;
}

bool TaskRos::asyncMode() const
{
    return _async;
}

TaskDescription::Ptr TaskRos::MakeInstance(std::string name,
                                           std::string type,
                                           ros::NodeHandle nh)
{
    if(type == "Cartesian")
    {
        return std::make_shared<CartesianRos>(name, nh);
    }
    else if(type == "Postural")
    {
        return std::make_shared<PosturalRos>(name, nh);
    }
    else
    {
        return std::make_shared<TaskRos>(name, nh);
    }
}

cartesian_interface::GetTaskInfoResponse TaskRos::get_task_info() const
{
    if(asyncMode())
    {
        cartesian_interface::GetTaskInfoResponse res;
        res.name = _info.name;
        res.size = _info.size;
        res.type = _info.type;
        res.lambda = _info.lambda;
        res.weight = _info.weight;
        res.indices = _info.indices;
        res.disabled_joints = _info.disabled_joints;
        res.activation_state = _info.activation_state;

        return res;
    }

    cartesian_interface::GetTaskInfo srv;
    if(!_task_prop_cli.call(srv))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             _task_prop_cli.getService()));
    }

    return srv.response;

}

void TaskRos::on_task_info_recv(cartesian_interface::TaskInfoConstPtr msg)
{
    _info = *msg;
}

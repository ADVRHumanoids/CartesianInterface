#include "ros/client_api/TaskRos.h"
#include "fmt/format.h"
#include "utils/DynamicLoading.h"

#include "../utils/RosUtils.h"

// #include "ros/client_api/CartesianRos.h"
// #include "ros/client_api/InteractionRos.h"
// #include "ros/client_api/PosturalRos.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ClientApi;
using namespace std::chrono_literals;

TaskRos::TaskRos(std::string name,
                 rclcpp::Node::SharedPtr node):
    _node(node),
    _name(name),
    _async(false)
{
    // task property getter service
    _task_prop_cli = _node->create_client<GetTaskInfo>(name + "/get_task_properties");

    while(!_task_prop_cli->wait_for_service(1s))
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fmt::format("waiting for service '{}'",
                                       _task_prop_cli->get_service_name())
                           );
    }

    // task property setter services
    _set_lambda_cli = _node->create_client<SetLambda>(name + "/set_lambda");
    _set_lambda2_cli = _node->create_client<SetLambda2>(name + "/set_lambda2");
    _set_weight_cli = _node->create_client<SetWeight>(name + "/set_weight");
    _activate_cli = _node->create_client<SetTaskActive>(name + "/set_active");

    //
    _task_changed_sub = ::create_subscription<std_msgs::msg::String>(
        _node,
        name + "/task_changed_event",
        &TaskRos::on_task_changed_ev_recv, this,
        10);

    // task property getter subscriber
    _task_info_sub = ::create_subscription<TaskInfo>(
        _node,
        name + "/task_properties",
        &TaskRos::on_task_info_recv, this,
        10);

}

bool TaskRos::validate()
{
    if(_info.name.empty())
    {
        return false;
    }

    return true;
}

void TaskRos::update(double time, double period)
{

}

void TaskRos::reset()
{

}

void TaskRos::log(XBot::MatLogger2::Ptr logger, bool init_logger, int buf_size)
{

}


const Eigen::MatrixXd& TaskRos::getWeight() const
{
    int size = getSize();

    auto res = get_task_info();
    _weight = Eigen::MatrixXf::Map(res.weight.data(),
                                   size, size).cast<double>();
    return _weight;
}

bool TaskRos::setWeight(const Eigen::MatrixXd& value)
{
    int size = getSize();

    // check weight size
    if(value.rows() != size || value.cols() != size)
    {
        Logger::error("%s \n",
                      fmt::format("invalid weight size ({} x {}), expected {} x {}",
                                  value.rows(), value.cols(),
                                  size, size).c_str());

        return false;
    }

    // fill in request
    auto req = std::make_shared<SetWeight::Request>();
    req->weight.resize(size*size);
    Eigen::MatrixXf::Map(req->weight.data(),
                         size, size) = value.cast<float>();

    auto fut = _set_weight_cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(),
                           fut.get()->message.c_str());
    }
    else
    {
        RCLCPP_ERROR(_node->get_logger(),
                     "service %s failed",
                     _set_weight_cli->get_service_name());

        return false;
    }

    return fut.get()->success;

}

const std::vector<int>& TaskRos::getIndices() const
{
    auto res = get_task_info();

    _indices.assign(res.indices.begin(),
                    res.indices.end());

    return _indices;
}

void TaskRos::setIndices(const std::vector<int>& value)
{
    throw std::runtime_error(fmt::format("Unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

double TaskRos::getLambda() const
{
    auto res = get_task_info();

    return res.lambda1;
}

void TaskRos::setLambda(double value)
{
    auto req = std::make_shared<SetLambda::Request>();

    req->lambda1 = value;

    auto fut = _set_lambda_cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             _set_lambda_cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             _set_lambda_cli->get_service_name()));
    }

}

double TaskRos::getLambda2() const
{
    auto res = get_task_info();

    return res.lambda2;
}

bool TaskRos::setLambda2(double value)
{
    auto req = std::make_shared<SetLambda2::Request>();

    auto cli = _set_lambda2_cli;

    req->lambda2 = value;

    req->auto_lambda2 = (value == -1.0);

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return true;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}

const std::vector<std::string>& TaskRos::getDisabledJoints() const
{
    throw std::runtime_error(fmt::format("unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

void TaskRos::setDisabledJoints(const std::vector<std::string>& value)
{
    throw std::runtime_error(fmt::format("unsupported function '{}'",
                                         __PRETTY_FUNCTION__));
}

ActivationState TaskRos::getActivationState() const
{
    auto res = get_task_info();

    if(res.activation_state == "Enabled") return ActivationState::Enabled;
    else return ActivationState::Disabled;
}

bool TaskRos::setActivationState(const ActivationState& value)
{
    auto req = std::make_shared<SetTaskActive::Request>();

    auto cli = _activate_cli;

    req->activation_state = (value == ActivationState::Enabled);

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO_STREAM(_node->get_logger(), fut.get()->message);

        if(fut.get()->success)
        {
            return true;
        }

        throw std::runtime_error(fmt::format("service '{}' returned false: {}",
                                             cli->get_service_name(),
                                             fut.get()->message));
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }
}


const std::string& TaskRos::getName() const
{
    return _name;
}

const std::string& TaskRos::getType() const
{
    _type = get_task_info().type.front();
    return _type;
}

int TaskRos::getSize() const
{
    return get_task_info().size;
}

const std::string& TaskRos::getLibName() const
{
    thread_local std::string ret;
    ret = get_task_info().lib_name;
    return ret;
}

void TaskRos::registerObserver(TaskObserver::WeakPtr obs)
{
    _observers.push_back(obs);
}

void TaskRos::setAsyncMode(bool is_async)
{
    // set async
    _async = is_async;

}

bool TaskRos::asyncMode() const
{
    return _async;
}


GetTaskInfo::Response TaskRos::get_task_info() const
{
    if(asyncMode())
    {
        GetTaskInfo::Response res;
        res.name = _info.name;
        res.size = _info.size;
        res.type = _info.type;
        res.lambda1 = _info.lambda1;
        res.lambda2 = _info.lambda2;
        res.weight = _info.weight;
        res.indices = _info.indices;
        res.disabled_joints = _info.disabled_joints;
        res.activation_state = _info.activation_state;

        return res;
    }

    auto req = std::make_shared<GetTaskInfo::Request>();

    auto cli = _task_prop_cli;

    auto fut = cli->async_send_request(req);

    if(rclcpp::spin_until_future_complete(_node, fut, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return *fut.get();
    }
    else
    {
        throw std::runtime_error(fmt::format("unable to call service '{}'",
                                             cli->get_service_name()));
    }

}

void TaskRos::on_task_info_recv(TaskInfo::ConstSharedPtr msg)
{
    _info = *msg;
}

void TaskRos::on_task_changed_ev_recv(std_msgs::msg::String::ConstSharedPtr msg)
{
    notifyTaskChanged(msg->data);
}

TaskDescription::Ptr LoadFromLib(std::string name,
                                 std::string type,
                                 std::string lib_name,
                                 rclcpp::Node::SharedPtr node)
{
    if(lib_name.empty())
    {
        return TaskDescription::Ptr();
    }

    try
    {
        auto task_rawptr = CallFunction<TaskRos*>(lib_name,
                                                  "create_cartesio_" + type + "_ros_client_api",
                                                  name,
                                                  node,
                                                  detail::Version CARTESIO_ABI_VERSION);

        return TaskDescription::Ptr(task_rawptr);
    }
    catch(LibNotFound&)
    {
        fmt::print("Unable to construct TaskRos instance for task '{}': "
                   "lib '{}' not found for task type '{}' \n",
                   name, lib_name, type);

    }
    catch(SymbolNotFound&)
    {
        fmt::print("Unable to construct TaskRos instance for task '{}': "
                   "factory not found for task type '{}' \n",
                   name, type);

    }

    return nullptr;
}

TaskDescription::Ptr TaskRos::MakeInstance(std::string name,
                                           std::vector<std::string> type_list,
                                           std::string lib_name,
                                           rclcpp::Node::SharedPtr node)
{

    for(auto type : type_list)
    {
        if(auto task = LoadFromLib(name, type, lib_name, node))
        {
            return task;
        }
  //       else if(type == "Interaction")
        // {
        // 	return std::make_shared<InteractionRos>(name, nh);
        // }
  //       else if(type == "Cartesian")
  //       {
  //           return std::make_shared<CartesianRos>(name, nh);
  //       }
  //       else if(type == "Postural")
  //       {
  //           return std::make_shared<PosturalRos>(name, nh);
  //       }
        else
        {
            return std::make_shared<TaskRos>(name, node);
        }
    }

    return nullptr;

}

void TaskRos::notifyTaskChanged(const std::string& message)
{
    if(message == "ActivationState")
    {
        NOTIFY_OBSERVERS(ActivationState)
    }
}

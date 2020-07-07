#include "ros/server_api/TaskRos.h"
#include "utils/DynamicLoading.h"
#include "fmt/format.h"

#include "ros/server_api/CartesianRos.h"
#include "ros/server_api/PosturalRos.h"
#include "ros/server_api/InteractionRos.h"

#include <std_msgs/String.h>
#include <eigen_conversions/eigen_msg.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::ServerApi;

TaskRos::TaskRos(TaskDescription::Ptr task,
                 RosContext::Ptr context):
    _task(task),
    task_name(task->getName()),
    _model(context->ci_context()->model()),
    _ctx(context)
{
    _get_task_info_srv = _ctx->nh().advertiseService(
        task_name + "/get_task_properties",
        &TaskRos::get_task_info_cb, this);

    _set_lambda_srv = _ctx->nh().advertiseService(
        task_name + "/set_lambda",
        &TaskRos::set_lambda_cb, this);

    _set_lambda2_srv = _ctx->nh().advertiseService(
        task_name + "/set_lambda2",
        &TaskRos::set_lambda2_cb, this);

    _set_weight_srv = _ctx->nh().advertiseService(
        task_name + "/set_weight",
        &TaskRos::set_weight_cb, this);

    _set_active_srv = _ctx->nh().advertiseService(
        task_name + "/set_active",
        &TaskRos::set_active_cb, this);

    _task_changed_pub = _ctx->nh().advertise<std_msgs::String>(
        task_name + "/task_changed_event", 10);

    _task_err_pub = _ctx->nh().advertise<std_msgs::Float64MultiArray>(
        task_name + "/task_error", 10);

    _type_hierarchy.push_back("Task");
}

void TaskRos::run(ros::Time time)
{
    /* Publish task error */
    if(_task->getTaskError(_err))
    {
        std_msgs::Float64MultiArray msg;
        tf::matrixEigenToMsg(_err, msg);
        _task_err_pub.publish(msg);
    }
}

bool TaskRos::onActivationStateChanged()
{
    notifyTaskChanged("ActivationState");

    return true;
}


bool TaskRos::get_task_info_cb(cartesian_interface::GetTaskInfoRequest & req,
                               cartesian_interface::GetTaskInfoResponse & res)
{
    res.name = _task->getName();

    res.type.assign(_type_hierarchy.begin(), _type_hierarchy.end());

    res.size = _task->getSize();

    res.lambda = _task->getLambda();

    res.lambda2 = _task->getLambda2();

    res.indices.assign(_task->getIndices().begin(),
                       _task->getIndices().end());

    res.disabled_joints = _task->getDisabledJoints();

    res.activation_state = EnumToString(_task->getActivationState());

    res.weight.resize(res.size*res.size);
    Eigen::MatrixXf::Map(res.weight.data(),
                         res.size,
                         res.size) = _task->getWeight().cast<float>();

    res.lib_name = _task->getLibName();

    return true;
}

bool TaskRos::set_lambda_cb(cartesian_interface::SetLambdaRequest & req,
                            cartesian_interface::SetLambdaResponse & res)
{
    if(req.lambda >= 0.0 && req.lambda <= 1.0)
    {
        _task->setLambda(req.lambda);
        res.message = fmt::format("Successfully set lambda = {} to task '{}'",
                                  req.lambda, _task->getName());
        res.success = true;

        return true;
    }

    res.message = fmt::format("Unable to set lambda = {} to task '{}': value not in range [0, 1]",
                              req.lambda, _task->getName());
    res.success = false;

    return true;
}

bool TaskRos::set_lambda2_cb(cartesian_interface::SetLambda2Request& req, cartesian_interface::SetLambda2Response& res)
{
    if(req.auto_lambda2)
    {
        _task->setLambda2(-1);
        res.message = fmt::format("Successfully set lambda2 = {} to task '{}'",
                                  -1, _task->getName());
        res.success = true;

        return true;
    }

    if(req.lambda2 >= 0.0 && req.lambda2 <= 1.0)
    {
        _task->setLambda2(req.lambda2);
        res.message = fmt::format("Successfully set lambda2 = {} to task '{}'",
                                  req.lambda2, _task->getName());
        res.success = true;

        return true;
    }


    res.message = fmt::format("Unable to set lambda2 = {} to task '{}': value not in range [0, 1]",
                              req.lambda2, _task->getName());
    res.success = false;

    return true;
}

bool TaskRos::set_weight_cb(cartesian_interface::SetWeightRequest & req,
                            cartesian_interface::SetWeightResponse & res)
{
    Eigen::MatrixXd w;
    w.setIdentity(_task->getSize(), _task->getSize());

    if(req.weight.size() == 1)
    {
        w *= req.weight[0];
    }

    else if(req.weight.size() == _task->getSize())
    {
        w = Eigen::VectorXf::Map(req.weight.data(), req.weight.size()).cast<double>().asDiagonal();
    }

    else if(req.weight.size() == _task->getSize()*_task->getSize())
    {
        w = Eigen::MatrixXf::Map(req.weight.data(),
                                 _task->getSize(),
                                 _task->getSize()).cast<double>();
    }
    else
    {
        res.success = false;
        res.message = fmt::format("Unable to set weight to task '{}': size should be either 1, n, or n^2",
                                  _task->getName());
        return true;
    }

    _task->setWeight(w);

    res.success = true;
    res.message = fmt::format("Successfully set weight to task '{}'",
                              _task->getName());

    return true;

}

bool TaskRos::set_active_cb(cartesian_interface::SetTaskActiveRequest & req,
                            cartesian_interface::SetTaskActiveResponse & res)
{
    ActivationState req_state;

    if(req.activation_state)
    {
        req_state = ActivationState::Enabled;
    }
    else
    {
        req_state = ActivationState::Disabled;
    }

    res.success = _task->setActivationState(req_state);

    if(res.success)
    {
        res.message = fmt::format("Succesfully set activation state to '{}' for task '{}'",
                                  EnumToString(req_state), _task->getName());
    }
    else
    {
        res.message = fmt::format("Unable to set activation state to '{}' for task '{}'",
                                  EnumToString(req_state), _task->getName());
    }

    return true;
}

RosApiNotFound::RosApiNotFound(std::string message): _msg(message)
{}

const char * RosApiNotFound::what() const noexcept
{
    return _msg.c_str();
}


TaskRos::Ptr TaskRos::MakeInstance(TaskDescription::Ptr task,
                                   RosContext::Ptr context)
{
    TaskRos * ros_adapter = nullptr;

    /* Try all supported dynamic casts, from the most derived to the least derived class */
    if(auto inter = std::dynamic_pointer_cast<InteractionTask>(task))
    {
        ros_adapter = new InteractionRos(inter, context);
    }
    else if(auto cart = std::dynamic_pointer_cast<CartesianTask>(task))
    {
        ros_adapter = new CartesianRos(cart, context);
    }
    else if(auto post = std::dynamic_pointer_cast<PosturalTask>(task))
    {
        ros_adapter = new PosturalRos(post, context);
    }
    else if(!task->getLibName().empty()) /* Otherwise, construct plugin, or fallback to generic Task interface */
    {
        try
        {
            ros_adapter = CallFunction<TaskRos*>(task->getLibName(),
                                                  "create_cartesio_" + task->getType() + "_ros_api",
                                                  task,
                                                  context,
                                                  detail::Version CARTESIO_ABI_VERSION);
        }
        catch(LibNotFound&)
        {
            fmt::print("Unable to construct TaskRos instance for task '{}': "
                       "lib '{}' not found for task type '{}' \n",
                       task->getName(), task->getLibName(), task->getType());

        }
        catch(SymbolNotFound&)
        {
            fmt::print("Unable to construct TaskRos instance for task '{}': "
                       "factory not found for task type '{}' \n",
                       task->getName(), task->getType());

        }
    }

    if(!ros_adapter) ros_adapter = new TaskRos(task, context);

    Ptr rosapi_shared_ptr(ros_adapter);

    if(!rosapi_shared_ptr->initialize())
    {
        throw std::runtime_error(fmt::format("Unable to initialize TaskRos instance for task '{}'",
                                             task->getName()));
    }

    return rosapi_shared_ptr;
}

bool TaskRos::initialize()
{
    _task->registerObserver(shared_from_this());

    return true;
}

void TaskRos::notifyTaskChanged(const std::string & message)
{
    std_msgs::String msg;
    msg.data = message;
    _task_changed_pub.publish(msg);
}

void TaskRos::registerType(const std::string & type)
{
    _type_hierarchy.push_front(type);
}

std::string TaskRos::RosApiPluginName(TaskDescription::Ptr task)
{
    return "libcartesio_ros_api_" + task->getType() + ".so";
}

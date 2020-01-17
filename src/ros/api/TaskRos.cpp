#include "TaskRos.h"
#include "utils/DynamicLoading.h"
#include "fmt/format.h"

#include "CartesianRos.h"

using namespace XBot::Cartesian;

TaskRos::TaskRos(TaskDescription::Ptr task):
    _task(task),
    task_name(task->getName()),
    _model(task->getModel())
{
    _get_task_info_srv = _ctx.nh().advertiseService(task_name + "/get_task_properties",
                                                    &TaskRos::get_task_info_cb, this);

    _set_lambda_srv = _ctx.nh().advertiseService(task_name + "/set_lambda",
                                                 &TaskRos::set_lambda_cb, this);

    _set_weight_srv = _ctx.nh().advertiseService(task_name + "/set_weight",
                                                 &TaskRos::set_weight_cb, this);

    _set_active_srv = _ctx.nh().advertiseService(task_name + "/set_active",
                                                 &TaskRos::set_active_cb, this);
}

void XBot::Cartesian::TaskRos::run(ros::Time time)
{
}


bool TaskRos::get_task_info_cb(cartesian_interface::GetTaskInfoRequest & req,
                               cartesian_interface::GetTaskInfoResponse & res)
{
    res.name = _task->getName();

    res.type = _task->getType();

    res.size = _task->getSize();

    res.lambda = _task->getLambda();

    res.indices.assign(_task->getIndices().begin(),
                       _task->getIndices().end());

    res.disabled_joints = _task->getDisabledJoints();

    res.activation_state = EnumToString(_task->getActivationState());

    res.weight.resize(res.size*res.size);
    Eigen::MatrixXf::Map(res.weight.data(),
                         res.size,
                         res.size) = _task->getWeight().cast<float>();

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


TaskRos::Ptr TaskRos::MakeInstance(TaskDescription::Ptr task)
{
    TaskRos * ros_adapter = nullptr;

    /* If lib name specified, load factory from plugin */
    if(task->getType() == "Cartesian") /* Otherwise, construct supported tasks */
    {
        ros_adapter = new CartesianRos(task);
    }
    //    else if(task->getType() == "Postural")
    //    {
    //    }
    //    else if(task->getType() == "Com")
    //    {
    //    }
    else
    {
        try
        {
            ros_adapter = CallFunction<TaskRos*>(RosApiPluginName(task),
                                                 "create_cartesio_ros_api",
                                                 task);
        }
        catch(LibNotFound&)
        {
            auto str = fmt::format("Unable to construct TaskRos instance for task '{}': "
                                   "lib '{}' not found for unsupported task type '{}'",
                                   task->getName(), RosApiPluginName(task), task->getType());

            throw RosApiNotFound(str);
        }
    }

    Ptr rosapi_shared_ptr(ros_adapter);

    return rosapi_shared_ptr;
}

std::string TaskRos::RosApiPluginName(TaskDescription::Ptr task)
{
    return "libcartesio_ros_api_" + task->getType() + ".so";
}

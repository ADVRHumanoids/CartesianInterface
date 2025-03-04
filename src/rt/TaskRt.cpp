#include "utils/DynamicLoading.h"
#include "fmt/format.h"

#include "rt/TaskRt.h"
#include "rt/CartesianRt.h"
#include "rt/CInteractionRt.h"

using namespace XBot::Cartesian;
namespace pl = std::placeholders;

TaskRt::TaskRt(TaskDescription::Ptr task):
    _task_impl(task)
{
    TaskRt::sendState(false);
    _to_cli_queue.reset(_rt_data);
    TaskRt::sendState(true);
    TaskRt::update(0, 0);
}


bool TaskRt::validate()
{
    return true;
}

const std::string & TaskRt::getName() const
{
    return _cli_data._name;
}

const std::string & TaskRt::getType() const
{
    return _cli_data._type;
}

int TaskRt::getSize() const
{
    return _cli_data._size;
}

const std::string & TaskRt::getLibName() const
{
    return _cli_data._lib_name;
}

const Eigen::MatrixXd & TaskRt::getWeight() const
{
    return _cli_data._weight;
}

bool TaskRt::setWeight(const Eigen::MatrixXd & value)
{
    auto cb = std::bind(&TaskDescription::setWeight,
                        pl::_1, value);

    return _cb_queue.push(cb);
}

const std::vector<int> & TaskRt::getIndices() const
{
    return _cli_data._indices;
}

void TaskRt::setIndices(const std::vector<int> & value)
{
    auto cb = std::bind(&TaskDescription::setIndices,
                        pl::_1, value);

    _cb_queue.push(cb);
}

double TaskRt::getLambda() const
{
    return _cli_data._lambda;
}

void TaskRt::setLambda(double value)
{
    auto cb = std::bind(&TaskDescription::setLambda,
                        pl::_1, value);

    _cb_queue.push(cb);
}

double TaskRt::getLambda2() const
{
    return _cli_data._lambda2;
}

bool TaskRt::setLambda2(double value)
{
    auto cb = std::bind(&TaskDescription::setLambda2,
                        pl::_1, value);

    _cb_queue.push(cb);
    return true;
}

const std::vector<std::string> & TaskRt::getDisabledJoints() const
{
    return _cli_data._disabled_joints;
}

void TaskRt::setDisabledJoints(const std::vector<std::string> & value)
{
    auto cb = std::bind(&TaskDescription::setDisabledJoints,
                        pl::_1, value);

    _cb_queue.push(cb);
}

void TaskRt::update(double time, double period)
{
    while(_to_cli_queue.pop(_cli_data));
}

void TaskRt::reset()
{
    auto cb = std::bind(&TaskDescription::reset,
                        pl::_1);

    _cb_queue.push(cb);
}

ActivationState TaskRt::getActivationState() const
{
    return _cli_data._activ_state;
}

bool TaskRt::setActivationState(const ActivationState & value)
{
    auto cb = std::bind(&TaskDescription::setActivationState,
                        pl::_1, value);

    return _cb_queue.push(cb);
}

void TaskRt::registerObserver(TaskObserver::WeakPtr obs)
{
}

void TaskRt::log(XBot::MatLogger2::Ptr logger, bool init_logger, int buf_size)
{
}

void TaskRt::callAvailable()
{
    _cb_queue.consume_all([this](CallbackType& cb)
    {
        cb(*_task_impl);
    });
}

void TaskRt::sendState(bool send)
{
    _rt_data._name = _task_impl->getName();
    _rt_data._size = _task_impl->getSize();
    _rt_data._type = _task_impl->getType();
    _rt_data._lib_name = _task_impl->getLibName();
    _rt_data._lambda = _task_impl->getLambda();
    _rt_data._lambda2 = _task_impl->getLambda2();
    _rt_data._weight = _task_impl->getWeight();
    _rt_data._indices = _task_impl->getIndices();
    _rt_data._activ_state = _task_impl->getActivationState();

    if(send) _to_cli_queue.push(_rt_data);
}

TaskRt::Ptr TaskRt::MakeInstance(TaskDescription::Ptr task)
{
    if(auto cint = std::dynamic_pointer_cast<InteractionTask>(task))
    {
        return std::make_shared<InteractionRt>(cint);
    }
    else if(auto cart = std::dynamic_pointer_cast<CartesianTask>(task))
    {
        return std::make_shared<CartesianRt>(cart);
    }
    else if(!task->getLibName().empty()) /* Otherwise, construct plugin, or fallback to generic Task interface */
    {
        TaskRt * task_rt = nullptr;
        try
        {
            task_rt = CallFunction<TaskRt*>(task->getLibName(),
                                                "create_cartesio_" + task->getType() + "_rt_api",
                                                task,
                                                detail::Version CARTESIO_ABI_VERSION); 
        }
        catch(LibNotFound&)
        {
            fmt::print("Unable to construct TaskRt instance for task '{}': "
                       "lib '{}' not found for task type '{}' \n",
                       task->getName(), task->getLibName(), task->getType());

        }
        catch(SymbolNotFound&)
        {
            fmt::print("Unable to construct TaskRt instance for task '{}': "
                       "factory not found for task type '{}' \n",
                       task->getName(), task->getType());

        }

        Ptr shared_task_rt(task_rt);
        return shared_task_rt;
    }
    else
    {
        return std::make_shared<TaskRt>(task);
    }
}

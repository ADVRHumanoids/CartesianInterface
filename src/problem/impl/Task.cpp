#include "problem/Task.h"

using namespace XBot::Cartesian;

AggregatedTask XBot::Cartesian::operator+(AggregatedTask task_1, TaskDescription::Ptr task_2)
{
    task_1.push_back(task_2);
    return task_1;
}

AggregatedTask XBot::Cartesian::operator+(AggregatedTask task_1, AggregatedTask task_2)
{
    task_1.insert(task_1.end(), task_2.begin(), task_2.end());
    return task_1;
}

AggregatedTask XBot::Cartesian::operator+(TaskDescription::Ptr task_1, AggregatedTask task_2)
{
    return operator+(task_2, task_1);
}

AggregatedTask XBot::Cartesian::operator+(TaskDescription::Ptr task_1, TaskDescription::Ptr task_2)
{
    AggregatedTask task = {task_1, task_2};
    return task;
}

Stack XBot::Cartesian::operator/(AggregatedTask task_1, AggregatedTask task_2)
{
    Stack stack = {task_1, task_2};
    return stack;
}

Stack XBot::Cartesian::operator/(TaskDescription::Ptr task_1, TaskDescription::Ptr task_2)
{
    return operator/(AggregatedTask(1, task_1), AggregatedTask(1, task_2));
}

Stack XBot::Cartesian::operator/(AggregatedTask task_1, TaskDescription::Ptr task_2)
{
    return operator/(task_1, AggregatedTask(1, task_2));
}

Stack XBot::Cartesian::operator/(TaskDescription::Ptr task_1, AggregatedTask task_2)
{
    return operator/(AggregatedTask(1, task_1), task_2);
}

TaskDescription::Ptr XBot::Cartesian::operator*(Eigen::Ref<const Eigen::MatrixXd> weight, 
                                                TaskDescription::Ptr task)
{
    if(weight.rows() != task->getSize())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    if(weight.cols() != task->getSize())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    task->setWeight(weight*task->getWeight());
    return task;
}

// TODO check logic
TaskDescription::Ptr XBot::Cartesian::operator%(std::vector<int> indices, TaskDescription::Ptr task)
{
    std::vector<int> new_indices;
    for(int idx : indices)
    {
        new_indices.push_back(task->getIndices()[idx]);
    }
    
    task->setIndices(new_indices);
    
    return task;
}



TaskDescriptionImpl::TaskDescriptionImpl(std::string type,
                                 std::string name,
                                 int size,
                                 XBot::ModelInterface::ConstPtr model):
    TaskDescriptionImpl()
{
    _type = type;
    _name = name;
    _size = size;
    _model = model;

    _weight.setIdentity(_size, _size);

    for(int i = 0; i < _size; i++)
    {
        _indices.push_back(i);
    }
}

TaskDescriptionImpl::TaskDescriptionImpl(YAML::Node task_node,
                                 XBot::ModelInterface::ConstPtr model,
                                 std::string name,
                                 int size):
    TaskDescriptionImpl(task_node["type"].as<std::string>(),
                    name,
                    size,
                    model)
{
    if(task_node["name"])
    {
        _name = task_node["name"].as<std::string>();
    }

    if(task_node["active"] && !task_node["active"].as<bool>())
    {
        setActivationState(ActivationState::Disabled);
    }

    if(task_node["weight"] && task_node["weight"].IsScalar())
    {
        _weight = task_node["weight"].as<double>() * Eigen::MatrixXd::Identity(_size, _size);
    }

    if(task_node["weight"] && task_node["weight"].IsSequence())
    {
        auto w_vec = task_node["weight"].as<std::vector<double>>();
        if(w_vec.size() != _size)
        {
            throw std::invalid_argument("weight size does not match task size");
        }
        _weight = Eigen::VectorXd::Map(w_vec.data(), w_vec.size()).asDiagonal();
    }

    if(task_node["lib_name"])
    {
        _lib_name = task_node["lib_name"].as<std::string>();
    }

    if(task_node["lambda"])
    {
        _lambda = task_node["lambda"].as<double>();
    }

    if(task_node["lambda2"])
    {
        _lambda2 = task_node["lambda2"].as<double>();
    }
    else
    {
        _lambda2 = -1.0;
    }

    if(task_node["indices"])
    {
        std::vector<int> indices = task_node["indices"].as<std::vector<int>>();
        setIndices(indices);
    }

    if(task_node["disabled_joints"])
    {
        for(auto jnode : task_node["disabled_joints"])
        {
            std::string jstr = jnode.as<std::string>();

            if(!model->hasJoint(jstr))
            {
                throw std::runtime_error("Undefined joint '" + jstr + "' listed among disabled joints");
            }

            _disabled_joints.push_back(jstr);
        }
    }

    if(task_node["enabled_joints"])
    {

        if(task_node["disabled_joints"])
        {
            throw std::runtime_error("Cannot specify both 'enabled_joints' and 'disabled_joints'");
        }

        _disabled_joints = model->getEnabledJointNames();

        for(auto jnode : task_node["enabled_joints"])
        {
            std::string jstr = jnode.as<std::string>();

            if(!model->hasJoint(jstr))
            {
                throw std::runtime_error("Undefined joint '" + jstr + "' listed among enabled joints");
            }

            auto it = std::find(_disabled_joints.begin(), _disabled_joints.end(), jstr);
            _disabled_joints.erase(it);
        }
    }
}

bool TaskDescriptionImpl::validate()
{
    bool ret = true;

    if(_lambda < 0)
    {
        Logger::error("Task '%s': invalid lambda = %f \n",
                                 getName().c_str(), _lambda);

        ret = false;
    }

    if(!_weight.isApprox(_weight.transpose()) &&
            _weight.llt().info() == Eigen::NumericalIssue)
    {
        Logger::error("Task '%s': invalid weight \n",
                                 getName().c_str());

        ret = false;
    }

    return ret;
}

const std::string& TaskDescriptionImpl::getType() const
{
    return _type;
}

ActivationState TaskDescriptionImpl::getActivationState() const
{
    return _activ_state;
}

bool TaskDescriptionImpl::setActivationState(const ActivationState & value)
{
    _activ_state = value;
    reset();

    NOTIFY_OBSERVERS(ActivationState);

    return true;
}

void TaskDescriptionImpl::registerObserver(std::weak_ptr<TaskObserver> obs)
{
    _observers.push_back(obs);
}

void TaskDescriptionImpl::log(MatLogger::Ptr logger, bool init_logger, int buf_size)
{
    if(init_logger)
    {
        logger->createScalarVariable(getName() + "_active", 1, buf_size);
        return;
    }

    logger->add(getName() + "_active", _activ_state == ActivationState::Enabled);
}

XBot::ModelInterface::ConstPtr TaskDescriptionImpl::getModel() const
{
    return _model;
}

void TaskDescriptionImpl::setLibName(std::string __lib_name)
{
    _lib_name = __lib_name;
}

double TaskDescriptionImpl::getTime() const
{
    return _time;
}

TaskDescriptionImpl::TaskDescriptionImpl():
    _activ_state(ActivationState::Enabled),
    _size(-1),
    _lambda(1.0),
    _lambda2(-1.0),
    _time(0.)
{

}

const std::string& TaskDescriptionImpl::getName() const
{
    return _name;
}

int TaskDescriptionImpl::getSize() const
{
    return _size;
}

const std::string& TaskDescriptionImpl::getLibName() const
{
    return _lib_name;
}

const Eigen::MatrixXd& TaskDescriptionImpl::getWeight() const
{
    return _weight;
}

bool TaskDescriptionImpl::setWeight(const Eigen::MatrixXd & value)
{
    _weight = value;

    NOTIFY_OBSERVERS(Weight)

    return true;
}

const std::vector<int>& TaskDescriptionImpl::getIndices() const
{
    return _indices;
}

void TaskDescriptionImpl::setIndices(const std::vector<int> & value)
{
    auto invalid_idx_predicate = [this](int idx){ return idx >= _size || idx < 0; };
    if(std::any_of(value.begin(), value.end(), invalid_idx_predicate))
    {
        throw std::out_of_range("indices out of range");
    }

    _indices = value;
}

double TaskDescriptionImpl::getLambda() const
{
    return _lambda;
}

void TaskDescriptionImpl::setLambda(double value)
{
    _lambda = value;
}

double TaskDescriptionImpl::getLambda2() const
{
    return _lambda2;
}

bool TaskDescriptionImpl::setLambda2(double value)
{
    _lambda2 = value;
    return true;
}

const std::vector<std::string>& TaskDescriptionImpl::getDisabledJoints() const
{
    return _disabled_joints;
}

void TaskDescriptionImpl::setDisabledJoints(const std::vector<std::string> & value)
{
    _disabled_joints = value;
}

void TaskDescriptionImpl::update(double time, double period)
{
    _time = time;
}

void TaskDescriptionImpl::reset()
{

}



bool TaskObserver::onWeightChanged() { return true; }

bool TaskObserver::onActivationStateChanged() { return true; }

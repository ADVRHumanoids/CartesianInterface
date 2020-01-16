#include <cartesian_interface/problem/Task.h>


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
    if(weight.rows() != task->_weight.rows())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    if(weight.cols() != task->_weight.cols())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    task->_weight = weight*task->_weight;
    return task;
}

// TODO check logic
TaskDescription::Ptr XBot::Cartesian::operator%(std::vector<int> indices, TaskDescription::Ptr task)
{
    std::vector<int> new_indices;
    for(int idx : indices)
    {
        new_indices.push_back(task->_indices[idx]);
    }
    
    task->_indices = new_indices;
    Eigen::MatrixXd new_weight(new_indices.size(), task->_weight.cols());
    
    {
        int i = 0;
        for(uint idx : indices)
        {
            new_weight.row(i) = task->_weight.row(idx);
            i++;
        }
    }
    
    task->_weight.resize(indices.size(), indices.size());
    
    {
        int i = 0;
        for(uint idx : indices)
        {
            task->_weight.col(i) = new_weight.col(idx);
            i++;
        }
    }
    
    return task;
}

TaskDescription::TaskDescription(std::string type,
                                 std::string name,
                                 int size,
                                 ModelInterface::ConstPtr model):
    TaskDescription()
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

TaskDescription::TaskDescription(YAML::Node task_node,
                                 XBot::ModelInterface::ConstPtr model,
                                 std::string name,
                                 int size):
    TaskDescription(task_node["type"].as<std::string>(),
                    name,
                    size,
                    model)
{

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
        _lambda2 = 0.;
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

bool TaskDescription::validate()
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

std::string TaskDescription::getType() const
{
    return _type;
}

ActivationState TaskDescription::getActivationState() const
{
    return _activ_state;
}

bool TaskDescription::setActivationState(const ActivationState & value)
{
    _activ_state = value;
    reset();

    NOTIFY_OBSERVERS(ActivationState);

    return true;
}

void TaskDescription::registerObserver(std::weak_ptr<TaskObserver> obs)
{
    _observers.push_back(obs);
}

void TaskDescription::log(MatLogger::Ptr logger, bool init_logger, int buf_size)
{
    if(init_logger)
    {
        logger->createScalarVariable(getName() + "_active", buf_size);
        return;
    }

    logger->add(getName() + "_active", _activ_state == ActivationState::Enabled);
}

XBot::ModelInterface::ConstPtr TaskDescription::getModel() const
{
    return _model;
}

void TaskDescription::setLibName(std::string __lib_name)
{
    _lib_name = __lib_name;
}

double TaskDescription::getTime() const
{
    return _time;
}

TaskDescription::TaskDescription():
    _activ_state(ActivationState::Enabled),
    _size(-1),
    _lambda(1.0),
    _lambda2(0.0),
    _time(0.)
{

}

const std::string& TaskDescription::getName() const
{
    return _name;
}

int TaskDescription::getSize() const
{
    return _size;
}

const std::string& TaskDescription::getLibName() const
{
    return _lib_name;
}

const Eigen::MatrixXd& TaskDescription::getWeight() const
{
    return _weight;
}

bool TaskDescription::setWeight(const Eigen::MatrixXd & value)
{
    _weight = value;

    NOTIFY_OBSERVERS(Weight)

    return true;
}

const std::vector<int>& TaskDescription::getIndices() const
{
    return _indices;
}

void TaskDescription::setIndices(const std::vector<int> & value)
{
    auto invalid_idx_predicate = [this](int idx){ return idx >= _size || idx < 0; };
    if(std::any_of(value.begin(), value.end(), invalid_idx_predicate))
    {
        throw std::out_of_range("indices out of range");
    }

    _indices = value;
}

double TaskDescription::getLambda() const
{
    return _lambda;
}

void TaskDescription::setLambda(double value)
{
    _lambda = value;
}

const std::vector<std::string>& TaskDescription::getDisabledJoints() const
{
    return _disabled_joints;
}

void TaskDescription::setDisabledJoints(const std::vector<std::string> & value)
{
    _disabled_joints = value;
}

void TaskDescription::update(double time, double period)
{
    _time = time;
}

void TaskDescription::reset()
{

}



bool TaskObserver::onWeightChanged() { return true; }

bool TaskObserver::onActivationStateChanged() { return true; }

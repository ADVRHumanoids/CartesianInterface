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
    if(weight.rows() != task->weight.rows())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    if(weight.cols() != task->weight.cols())
    {
        throw std::invalid_argument("weight matrix size mismatch");
    }
    
    task->weight = weight*task->weight;
    return task;
}

// TODO check logic
TaskDescription::Ptr XBot::Cartesian::operator%(std::vector<int> indices, TaskDescription::Ptr task)
{
    std::vector<int> new_indices;
    for(int idx : indices)
    {
        new_indices.push_back(task->indices[idx]);
    }
    
    task->indices = new_indices;
    Eigen::MatrixXd new_weight(new_indices.size(), task->weight.cols());
    
    {
        int i = 0;
        for(uint idx : indices)
        {
            new_weight.row(i) = task->weight.row(idx);
            i++;
        }
    }
    
    task->weight.resize(indices.size(), indices.size());
    
    {
        int i = 0;
        for(uint idx : indices)
        {
            task->weight.col(i) = new_weight.col(idx);
            i++;
        }
    }
    
    return task;
}

TaskDescription::TaskDescription(std::string __type, std::string __name, int __size, ModelInterface::ConstPtr model):
    weight(Eigen::MatrixXd::Identity(size,size)),
    lambda(1.0),
    lambda2(0.0),
    name(__name),
    type(__type),
    size(__size),
    _model(model),
    activ_state(ActivationState::Enabled),
    _time(0.)
{
    for(int i = 0; i < size; i++)
    {
        indices.push_back(i);
    }
}

TaskDescription::TaskDescription(YAML::Node task_node,
                                 XBot::ModelInterface::ConstPtr model,
                                 std::string __name,
                                 int __size):
    name(__name),
    size(__size),
    _model(model),
    activ_state(ActivationState::Enabled),
    _time(0.),
    lambda(1.0),
    lambda2(0.0)
{
    type = task_node["type"].as<std::string>();

    if(task_node["weight"] && task_node["weight"].IsScalar())
    {
        weight = task_node["weight"].as<double>() * Eigen::MatrixXd::Identity(size, size);
    }

    if(task_node["weight"] && task_node["weight"].IsSequence())
    {
        auto w_vec = task_node["weight"].as<std::vector<double>>();
        if(w_vec.size() != size)
        {
            throw std::invalid_argument("weight size does not match task size");
        }
        weight = Eigen::VectorXd::Map(w_vec.data(), w_vec.size()).asDiagonal();
    }

    if(task_node["lambda"])
    {
        lambda = task_node["lambda"].as<double>();
    }

    if(task_node["lambda2"])
    {
        lambda2 = task_node["lambda2"].as<double>();
    }
    else
    {
        lambda2 = -1.;
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

            disabled_joints.push_back(jstr);
        }
    }

    if(task_node["enabled_joints"])
    {

        if(task_node["disabled_joints"])
        {
            throw std::runtime_error("Cannot specify both 'enabled_joints' and 'disabled_joints'");
        }

        disabled_joints = model->getEnabledJointNames();

        for(auto jnode : task_node["enabled_joints"])
        {
            std::string jstr = jnode.as<std::string>();

            if(!model->hasJoint(jstr))
            {
                throw std::runtime_error("Undefined joint '" + jstr + "' listed among enabled joints");
            }

            auto it = std::find(disabled_joints.begin(), disabled_joints.end(), jstr);
            disabled_joints.erase(it);
        }
    }
}

bool TaskDescription::validate()
{
    bool ret = true;

    if(lambda < 0)
    {
        Logger::error("Task '%s': invalid lambda = %f \n",
                                 getName().c_str(), lambda);

        ret = false;
    }

    if(!weight.isApprox(weight.transpose()) &&
            weight.llt().info() == Eigen::NumericalIssue)
    {
        Logger::error("Task '%s': invalid weight \n",
                                 getName().c_str());

        ret = false;
    }

    return ret;
}

std::string TaskDescription::getType() const
{
    return type;
}

ActivationState TaskDescription::getActivationState() const
{
    return activ_state;
}

bool TaskDescription::setActivationState(const ActivationState & value)
{
    activ_state = value;
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

    logger->add(getName() + "_active", activ_state == ActivationState::Enabled);
}

XBot::ModelInterface::ConstPtr TaskDescription::getModel() const
{
    return _model;
}

void TaskDescription::setLibName(std::string __lib_name)
{
    lib_name = __lib_name;
}

double TaskDescription::getTime() const
{
    return _time;
}

const std::string& TaskDescription::getName() const
{
    return name;
}

int TaskDescription::getSize() const
{
    return size;
}

const std::string& TaskDescription::getLibName() const
{
    return lib_name;
}

const Eigen::MatrixXd& TaskDescription::getWeight() const
{
    return weight;
}

bool TaskDescription::setWeight(const Eigen::MatrixXd & value)
{
    weight = value;

    NOTIFY_OBSERVERS(Weight)

    return true;
}

const std::vector<int>& TaskDescription::getIndices() const
{
    return indices;
}

void TaskDescription::setIndices(const std::vector<int> & value)
{
    auto invalid_idx_predicate = [this](int idx){ return idx >= size || idx < 0; };
    if(std::any_of(value.begin(), value.end(), invalid_idx_predicate))
    {
        throw std::out_of_range("indices out of range");
    }

    indices = value;
}

double TaskDescription::getLambda() const
{
    return lambda;
}

void TaskDescription::setLambda(double value)
{
    lambda = value;
}

const std::vector<std::string>& TaskDescription::getDisabledJoints() const
{
    return disabled_joints;
}

void TaskDescription::setDisabledJoints(const std::vector<std::string> & value)
{
    disabled_joints = value;
}

void TaskDescription::update(double time, double period)
{
    _time = time;
}



bool TaskObserver::onWeightChanged() { return true; }

bool TaskObserver::onActivationStateChanged() { return true; }

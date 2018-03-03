#include <CartesianInterface/ProblemDescription.h>

using namespace XBot::Cartesian;

ProblemDescription::ProblemDescription(AggregatedTask task):
    ProblemDescription(Stack(1, task))
{
    
}

ProblemDescription::ProblemDescription(TaskDescription::Ptr task):
    ProblemDescription(AggregatedTask(1, task))
{

}

ProblemDescription::ProblemDescription(Stack stack):
    _stack(stack)
{

}


ProblemDescription& ProblemDescription::operator<<(ConstraintDescription::Ptr constraint)
{
    _bounds.push_back(constraint);
    return *this;
}


CartesianTask::Ptr XBot::Cartesian::MakeCartesian(std::string distal_link, std::string base_link)
{
    return std::make_shared<CartesianTask>(distal_link, base_link);
}

CartesianTask::Ptr XBot::Cartesian::GetAsCartesian(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<CartesianTask>(task);
}

ConstraintDescription::Ptr XBot::Cartesian::MakeJointLimits()
{
    return std::make_shared<ConstraintDescription>(ConstraintType::JointLimits);
}

ConstraintDescription::Ptr XBot::Cartesian::MakeVelocityLimits()
{
    return std::make_shared<ConstraintDescription>(ConstraintType::VelocityLimits);
}


CartesianTask::CartesianTask(std::string distal_link, std::string base_link):
    TaskDescription(TaskType::Cartesian, 6),
    distal_link(distal_link),
    base_link(base_link),
    orientation_gain(0.1)
{

}

TaskDescription::TaskDescription(TaskType type, int size):
    type(type),
    weight(Eigen::MatrixXd::Identity(size,size)),
    lambda(1.0)
{
    for(uint i = 0; i < size; i++)
    {
        indices.push_back(i);
    }
}

ConstraintDescription::ConstraintDescription(ConstraintType type):
    type(type)
{

}

int ProblemDescription::getNumTasks() const
{
    return _stack.size();
}

AggregatedTask ProblemDescription::getTask(int id) const
{
    return _stack.at(id);
}

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

TaskDescription::Ptr XBot::Cartesian::operator*(Eigen::Ref<const Eigen::MatrixXd> weight, TaskDescription::Ptr task)
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

TaskDescription::Ptr operator%(TaskDescription::Ptr task, std::vector<int> indices)
{
    std::vector<int> new_indices;
    for(uint idx : indices)
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

const std::vector< ConstraintDescription::Ptr >& ProblemDescription::getBounds() const
{
    return _bounds;
}


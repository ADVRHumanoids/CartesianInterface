#include <cartesian_interface/problem/Task.h>


using namespace XBot::Cartesian;


TaskDescription::TaskDescription(TaskInterface ifc, std::string __type, int size):
    interface(ifc),
    weight(Eigen::MatrixXd::Identity(size,size)),
    lambda(1.0),
    type(__type)
{
    for(uint i = 0; i < size; i++)
    {
        indices.push_back(i);
    }
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

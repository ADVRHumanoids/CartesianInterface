#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/AngularMomentum.h>
#include <cartesian_interface/problem/Interaction.h>

using namespace XBot::Cartesian;

AngularMomentumTask::Ptr XBot::Cartesian::MakeAngularMomentum()
{
    return std::make_shared<AngularMomentumTask>();
}

AngularMomentumTask::Ptr XBot::Cartesian::GetAsAngularMomentum(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<AngularMomentumTask>(task);
}

XBot::Cartesian::AngularMomentumTask::AngularMomentumTask():
    TaskDescription(TaskInterface::None, "AngularMomentum", 3),
    min_rate(false)
{

}

GazeTask::Ptr XBot::Cartesian::MakeGaze(std::string base_link)
{
    return std::make_shared<GazeTask>(base_link);
}

GazeTask::Ptr XBot::Cartesian::GetAsGaze(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<GazeTask>(task);
}


CartesianTask::Ptr XBot::Cartesian::MakeCartesian(std::string distal_link, std::string base_link)
{
    return std::make_shared<CartesianTask>(distal_link, base_link);
}

CartesianTask::Ptr XBot::Cartesian::GetAsCartesian(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<CartesianTask>(task);
}

PosturalTask::Ptr XBot::Cartesian::GetAsPostural(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<PosturalTask>(task);
}

PosturalTask::PosturalTask(int ndof):
    TaskDescription(TaskInterface::Postural, "Postural", ndof),
    use_inertia_matrix(false)
{
    
}

PosturalTask::Ptr XBot::Cartesian::MakePostural(int ndof)
{
    return std::make_shared<PosturalTask>(ndof);
}


GazeTask::GazeTask(std::string base_link):
    CartesianTask("gaze", base_link, 2, "Gaze") //HERE THE SIZE IS 2 BECAUSE THE GAZE IS CONSIDERED IMPLEMENTED USING PAN-TILT
{

}

CartesianTask::CartesianTask(std::string distal_link, std::string base_link, int size, std::string type):
    TaskDescription(TaskInterface::Cartesian, type, size),
    distal_link(distal_link),
    base_link(base_link),
    orientation_gain(1.0)
{

}

InteractionTask::Ptr XBot::Cartesian::GetAsInteraction(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<InteractionTask>(task);
}

InteractionTask::InteractionTask(std::string distal_link, 
                                 std::string base_link,
                                 int size, 
                                 std::string type):
    CartesianTask(distal_link, base_link, 6, type)
{
    interface = TaskInterface::Interaction;
}

InteractionTask::Ptr XBot::Cartesian::MakeInteraction(std::string distal_link, 
                                                      std::string base_link)
{
    return std::make_shared<InteractionTask>(distal_link, base_link, 6, "Interaction");
}


ComTask::ComTask():
    CartesianTask("com", "world", 3, "Com")
{
    
}

ComTask::Ptr XBot::Cartesian::MakeCom()
{
    return std::make_shared<ComTask>();
}

ComTask::Ptr XBot::Cartesian::GetAsCom(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<ComTask>(task);
}


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

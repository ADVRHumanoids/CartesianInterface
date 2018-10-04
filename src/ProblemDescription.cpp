#include <cartesian_interface/ProblemDescription.h>
#include <XBotInterface/Logger.hpp>

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

PosturalTask::Ptr XBot::Cartesian::GetAsPostural(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<PosturalTask>(task);
}

PosturalTask::PosturalTask(int ndof):
    TaskDescription(TaskType::Postural, "postural", ndof)
{
    
}

PosturalTask::Ptr XBot::Cartesian::MakePostural(int ndof)
{
    return std::make_shared<PosturalTask>(ndof);
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
    TaskDescription(TaskType::Cartesian, distal_link, 6),
    distal_link(distal_link),
    base_link(base_link),
    orientation_gain(1.0)
{

}

ComTask::ComTask():
    TaskDescription(TaskType::Com, "com", 3)
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


TaskDescription::TaskDescription(TaskType type, std::string __name, int size):
    type(type),
    weight(Eigen::MatrixXd::Identity(size,size)),
    lambda(1.0),
    name(__name)
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

const std::vector< ConstraintDescription::Ptr >& ProblemDescription::getBounds() const
{
    return _bounds;
}

const YAML::Node& XBot::Cartesian::ProblemDescription::getSolverOptions() const
{
    return _solver_options_node;
}


ProblemDescription::ProblemDescription(YAML::Node yaml_node, ModelInterface::ConstPtr model)
{
    if(!yaml_node["stack"])
    {
        throw std::runtime_error("Missing node \"stack\"");
    }
    
    _solver_options_node = yaml_node["solver_options"];
    
    YAML::Node stack = yaml_node["stack"];
    
    for(auto stack_level : stack)
    {
        AggregatedTask aggr_task;
        
        for(auto task : stack_level)
        {
            
            std::string task_name = task.as<std::string>();
            std::string task_type = yaml_node[task_name]["type"].as<std::string>();
            TaskDescription::Ptr task_desc;
            
            if(task_type == "Cartesian") // TBD ERROR CHECKING
            {
                std::string distal_link = yaml_node[task_name]["distal_link"].as<std::string>();
                std::string base_link = "world";
                
                
                if(yaml_node[task_name]["base_link"])
                {
                    base_link = yaml_node[task_name]["base_link"].as<std::string>();
                }

                auto cart_task = MakeCartesian(distal_link, base_link);
                task_desc = cart_task;
                
                if(yaml_node[task_name]["orientation_gain"])
                {
                    cart_task->orientation_gain = yaml_node[task_name]["orientation_gain"].as<double>();
                }
            }
            else if(task_type == "Com")
            {
                task_desc = MakeCom();
            }
            else if(task_type == "Postural")
            {
                task_desc = MakePostural(model->getJointNum());
                auto postural_desc = GetAsPostural(task_desc);
                
                if(yaml_node[task_name]["weight"] && yaml_node[task_name]["weight"].IsMap())
                {
                    for(const auto& pair : yaml_node[task_name]["weight"])
                    {
                        if(!model->hasJoint(pair.first.as<std::string>()))
                        {
                            std::string err = "Joint " + pair.first.as<std::string>() + " is undefined";
                            throw std::invalid_argument(err);
                        }
                        
                        int idx = model->getDofIndex(pair.first.as<std::string>());
                        task_desc->weight(idx, idx) = pair.second.as<double>();
                        
                    }
                }
                    
            }
            else
            {
                XBot::Logger::error("Unsupported task type %s\n", task_type.c_str());
                throw std::runtime_error("Bad problem description");
            }

            
            if(yaml_node[task_name]["weight"] && yaml_node[task_name]["weight"].IsScalar())
            {
                task_desc->weight *= yaml_node[task_name]["weight"].as<double>();
            }
            
            if(yaml_node[task_name]["lambda"])
            {
                task_desc->lambda = yaml_node[task_name]["lambda"].as<double>();
            }
            
            if(yaml_node[task_name]["indices"])
            {
                std::vector<int> indices = yaml_node[task_name]["indices"].as<std::vector<int>>();
                task_desc = indices % task_desc;
            }
                
            
            aggr_task.push_back(task_desc);
            
        }
        
        _stack.push_back(aggr_task);
    }
    
    YAML::Node constraints = yaml_node["constraints"];
    
    if(constraints)
    {
        for(auto constr : constraints)
        {
            std::string constr_type = constr.as<std::string>();
            
            if(constr_type == "JointLimits")
            {
                _bounds.push_back(MakeJointLimits());
            }
            else if(constr_type == "VelocityLimits")
            {
                _bounds.push_back(MakeVelocityLimits());
            }
            else
            {
                XBot::Logger::error("Unsupported constraint type %s\n", constr_type.c_str());
                throw std::runtime_error("Bad problem description");
            }
            
        }
    }
    
    
    
    
}

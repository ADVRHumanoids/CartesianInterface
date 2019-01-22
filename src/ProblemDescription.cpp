#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/problem/AngularMomentum.h>
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


int ProblemDescription::getNumTasks() const
{
    return _stack.size();
}

AggregatedTask ProblemDescription::getTask(int id) const
{
    return _stack.at(id);
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
    
    if(_solver_options_node)
    {
        Logger::success("Solver options node found\n");
    }
    
    YAML::Node stack = yaml_node["stack"];
    
    for(auto stack_level : stack)
    {
        AggregatedTask aggr_task;
        
        for(auto task : stack_level)
        {
            
            std::string task_name = task.as<std::string>();
            
            if(!yaml_node[task_name])
            {
                throw std::runtime_error("problem description parsing failed: node '" + task_name + "' undefined");
            }
            
            auto task_desc = yaml_parse_task(yaml_node[task_name], model);
            
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
            else if(yaml_node[constr_type])
            {
                auto task = yaml_parse_task(yaml_node[constr_type], model);
                if(task)
                {
                    _bounds.push_back(MakeConstraintFromTask(task));
                }
                else
                {
                    XBot::Logger::error("Unsupported constraint type %s: not a task\n", constr_type.c_str());
                    throw std::runtime_error("Bad problem description");
                }
            }
            else
            {
                XBot::Logger::error("Unsupported constraint type %s\n", constr_type.c_str());
                throw std::runtime_error("Bad problem description");
            }
            
        }
    }
    
    
}

TaskDescription::Ptr XBot::Cartesian::ProblemDescription::yaml_parse_task(YAML::Node task_node,
                                                                          XBot::ModelInterface::ConstPtr model)
{
    
    std::string task_type = task_node["type"].as<std::string>();
    
    TaskDescription::Ptr task_desc;
            
    if(task_type == "Cartesian") // TBD ERROR CHECKING
    {
        task_desc = CartesianTask::yaml_parse_cartesian(task_node, model);
    }
    else if(task_type == "Interaction")
    {
        task_desc = InteractionTask::yaml_parse_interaction(task_node, model);
    }
    else if(task_type == "Com")
    {
        task_desc = ComTask::yaml_parse_com(task_node, model);
    }
    else if(task_type == "Postural")
    {
        task_desc = PosturalTask::yaml_parse_postural(task_node, model);
    }
    else if(task_type == "Gaze")
    {
        task_desc = GazeTask::yaml_parse_gaze(task_node, model);
    }
    else if(task_type == "AngularMomentum")
    {
        task_desc = AngularMomentumTask::yaml_parse_angular_momentum(task_node, model);
    }
    else
    {
        XBot::Logger::error("Unsupported task type %s\n", task_type.c_str());
        throw std::runtime_error("Bad problem description");
    }
        
    if(task_node["weight"] && task_node["weight"].IsScalar())
    {
        task_desc->weight *= task_node["weight"].as<double>();
    }
    
    if(task_node["lambda"])
    {
        task_desc->lambda = task_node["lambda"].as<double>();
    }
    
    if(task_node["indices"])
    {
        std::vector<int> indices = task_node["indices"].as<std::vector<int>>();
        task_desc = indices % task_desc;
    }
    
    return task_desc;
}


ConstraintDescription::Ptr ProblemDescription::yaml_parse_joint_pos_lims(YAML::Node node, 
                                                   ModelInterface::ConstPtr model)
{
    return MakeJointLimits();
}

ConstraintDescription::Ptr ProblemDescription::yaml_parse_joint_vel_lims(YAML::Node node, 
                                                   ModelInterface::ConstPtr model)
{
    return MakeVelocityLimits();
}



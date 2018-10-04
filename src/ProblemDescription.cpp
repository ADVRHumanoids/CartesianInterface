#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
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

ConstraintDescription::ConstraintDescription(std::string __type, TaskInterface ifc):
    interface(ifc),
    type(__type)
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
                task_desc = yaml_parse_cartesian(yaml_node[task_name], model);
            }
            else if(task_type == "Com")
            {
                task_desc = yaml_parse_com(yaml_node[task_name], model);
            }
            else if(task_type == "Postural")
            {
                task_desc = yaml_parse_postural(yaml_node[task_name], model);    
            }
            else if(task_type == "Gaze")
            {
                task_desc = yaml_parse_gaze(yaml_node[task_name], model);  
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



TaskDescription::Ptr ProblemDescription::yaml_parse_cartesian(YAML::Node task_node, 
                                                   ModelInterface::ConstPtr model)
{
    
    std::string distal_link = task_node["distal_link"].as<std::string>();
    std::string base_link = "world";
    
    auto cart_task = MakeCartesian(distal_link, base_link);
    TaskDescription::Ptr task_desc = cart_task;
    
    
    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }

    if(task_node["orientation_gain"])
    {
        cart_task->orientation_gain = task_node["orientation_gain"].as<double>();
    }
    
    return task_desc;
}

TaskDescription::Ptr ProblemDescription::yaml_parse_com(YAML::Node node, 
                                                   ModelInterface::ConstPtr model)
{
    return MakeCom();
}

TaskDescription::Ptr ProblemDescription::yaml_parse_gaze(YAML::Node task_node, 
                                                   ModelInterface::ConstPtr model)
{
    
    std::string base_link = "world";

    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }

    return MakeGaze(base_link);
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

TaskDescription::Ptr ProblemDescription::yaml_parse_postural(YAML::Node task_node, 
                                                   ModelInterface::ConstPtr model)
{
    auto task_desc = MakePostural(model->getJointNum());
    auto postural_desc = GetAsPostural(task_desc);
    
    if(task_node["weight"] && task_node["weight"].IsMap())
    {
        for(const auto& pair : task_node["weight"])
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
    
    return task_desc;
}


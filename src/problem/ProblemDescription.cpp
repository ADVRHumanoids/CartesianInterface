#include <cartesian_interface/problem/ProblemDescription.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/utils/TaskFactory.h>
#include <cartesian_interface/problem/MinJointVel.h>
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

bool ProblemDescription::validate()
{
    return true;
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

AggregatedTask ProblemDescription::getRegularizationTask() const
{
    return _regularisation;
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
        throw std::runtime_error("Missing node 'stack'");
    }
    
    _solver_options_node = yaml_node["solver_options"];
    
    if(_solver_options_node)
    {
        Logger::success("Solver options node found\n");
    }

    YAML::Node regularization = yaml_node["regularization"];

    if(regularization)
    {
        for(auto task : regularization)
        {
            std::string task_name = task.as<std::string>();

            if(!yaml_node[task_name])
            {
                throw std::runtime_error("problem description parsing failed: node '" + task_name + "' undefined");
            }

            if(!yaml_node[task_name]["type"])
            {
                throw std::runtime_error("problem description parsing failed: missing type for '" + task_name + "'");
            }

            std::string lib_name = "";

            if(yaml_node[task_name]["lib_name"])
            {
                lib_name = yaml_node[task_name]["lib_name"].as<std::string>();
            }

            auto task_desc = MakeTaskDescription(yaml_node[task_name],
                                                 model,
                                                 lib_name);

            _regularisation.push_back(task_desc);
        }
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
            
            if(!yaml_node[task_name]["type"])
            {
                throw std::runtime_error("problem description parsing failed: missing type for '" + task_name + "'");
            }
            
            std::string lib_name = "";
            
            if(yaml_node[task_name]["lib_name"])
            {
                lib_name = yaml_node[task_name]["lib_name"].as<std::string>();
            }
            
            auto task_desc = MakeTaskDescription(yaml_node[task_name],
                                                 model,
                                                 lib_name);
            
            aggr_task.push_back(task_desc);
            
        }
        
        _stack.push_back(aggr_task);
    }
    
    YAML::Node constraints = yaml_node["constraints"];
    
    if(constraints)
    {
        for(auto constr : constraints)
        {
            std::string constr_name = constr.as<std::string>();
            
            if(!yaml_node[constr_name])
            {
                throw std::runtime_error("problem description parsing failed: node '" + constr_name + "' undefined");
            }

            if(!yaml_node[constr_name]["type"])
            {
                throw std::runtime_error("problem description parsing failed: missing type for '" + constr_name + "'");
            }

            std::string lib_name = "";
            
            if(yaml_node[constr_name]["lib_name"])
            {
                lib_name = yaml_node[constr_name]["lib_name"].as<std::string>();
            }

            auto task = MakeTaskDescription(yaml_node[constr_name],
                                            model,
                                            lib_name);

            if(auto constr = ConstraintDescription::AsConstraint(task))
            {
                _bounds.push_back(constr);
            }
            else
            {
                _bounds.push_back(MakeConstraintFromTask(task));
            }

        }
    }
    
    
}

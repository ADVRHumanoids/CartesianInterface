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

#include "problem/Subtask.h"

#include "fmt/format.h"



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

bool ProblemDescription::validate(bool verbose) const
{
    bool ret = true;

    for(auto& aggr : _stack)
    {
        for(auto& t : aggr)
        {
            bool valid = t->validate();
            ret = ret && valid;
            if(!valid && verbose)
            {
                fmt::print("validate() returned false for task '{}' \n", t->getName());
            }
        }
    }

    for(auto& c : _bounds)
    {
        bool valid = c->validate();
        ret = ret && valid;
        if(!valid && verbose)
        {
            fmt::print("validate() returned false for constraint '{}' \n", c->getName());
        }
    }

    return ret;
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

    //    YAML::Node regularization = yaml_node["regularization"];

    //    if(regularization)
    //    {
    //        for(auto task : regularization)
    //        {
    //            std::string task_name = task.as<std::string>();

    //            if(!yaml_node[task_name])
    //            {
    //                throw std::runtime_error("problem description parsing failed: node '" + task_name + "' undefined");
    //            }

    //            if(!yaml_node[task_name]["type"])
    //            {
    //                throw std::runtime_error("problem description parsing failed: missing type for '" + task_name + "'");
    //            }

    //            std::string lib_name = "";

    //            if(yaml_node[task_name]["lib_name"])
    //            {
    //                lib_name = yaml_node[task_name]["lib_name"].as<std::string>();
    //            }

    //            auto task_desc = MakeTaskDescription(yaml_node[task_name],
    //                                                 model,
    //                                                 lib_name);

    //            _regularisation.push_back(task_desc);
    //        }
    //    }


    /* Parse stack */

    YAML::Node stack = yaml_node["stack"];

    TaskFactory factory(yaml_node, model);

    for(auto stack_level : stack)
    {
        AggregatedTask aggr_task;
        
        for(auto task : stack_level)
        {
            auto task_desc = factory.makeTask(task.as<std::string>());
            
            aggr_task.push_back(task_desc);
        }
        
        _stack.push_back(aggr_task);
    }

    

    /* Parse constraints */

    YAML::Node constraints = yaml_node["constraints"];
    
    if(constraints)
    {
        for(auto constr : constraints)
        {
            std::string constr_name = constr.as<std::string>();
            
            auto task = factory.makeTask(constr_name);

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

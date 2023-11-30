#include <cartesian_interface/utils/TaskFactory.h>

#include "utils/DynamicLoading.h"

#include "problem/Cartesian.h"
#include "problem/Interaction.h"
#include "problem/Com.h"
#include "problem/Postural.h"
#include "problem/Limits.h"

#include "problem/Subtask.h"

#include "fmt/format.h"

namespace XBot { namespace Cartesian { 

std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node prob_desc,
                                                     std::string task_name,
                                                     Context::ConstPtr context
                                                     )
{
    /* Get task node */
    if(!prob_desc[task_name])
    {
        throw BadTaskDescription("Node '" + task_name + "' undefined");
    }

    auto task_node = prob_desc[task_name];

    /* Get task type */
    if(!task_node["type"])
    {
        throw BadTaskDescription("Missing type for '" + task_name + "'");
    }

    std::string task_type = task_node["type"].as<std::string>();

    if(task_type == "Subtask")
    {
        if(!task_node["task"])
        {
            throw BadTaskDescription(fmt::format("Missing field 'task' for subtask '{}'",
                                                 task_name));
        }

        if(task_node["indices"] && task_node["remove_indices"])
        {
            throw BadTaskDescription(fmt::format("'indices' and 'remove_indices' can not be set contemporary"));
        }

        if(!task_node["indices"] && !task_node["remove_indices"])
        {
            throw BadTaskDescription(fmt::format("Missing both fields 'indices' and 'remove_indices' for subtask '{}'",
                                                 task_name));
        }


        TaskIsSubtask e;
        e.real_task_name = task_node["task"].as<std::string>();
        if(task_node["indices"])
            e.indices = task_node["indices"].as<std::vector<int>>();
        if(task_node["remove_indices"])
            e.remove_indices = task_node["remove_indices"].as<std::vector<int>>();

        throw e;
    }

    /* Get task lib name */
    std::string lib_name = "";

    if(task_node["lib_name"])
    {
        lib_name = task_node["lib_name"].as<std::string>();
    }

    /* Load task descripton from library */
    TaskDescription::Ptr task_desc;

    if(!lib_name.empty())
    {
        try
        {
            task_desc.reset( CallFunction<TaskDescription *>(lib_name,
                                                             "create_cartesio_" + task_type + "_description",
                                                             task_node,
                                                             context,
                                                             detail::Version CARTESIO_ABI_VERSION
                                                             ) );

            return task_desc;
        }
        catch(SymbolNotFound& e)
        {
            fmt::print("Unable to load task description from '{}' for task '{}', "
                       "trying with supported tasks.. \n", lib_name, task_type);
        }

    }

    if(task_type == "Cartesian") // TBD ERROR CHECKING
    {
        task_desc = std::make_shared<CartesianTaskImpl>(task_node,  context);
    }
    else if(task_type == "Interaction")
    {
        task_desc = std::make_shared<InteractionTaskImpl>(task_node,  context);
    }
    else if(task_type == "Admittance")
    {
        task_desc = std::make_shared<AdmittanceTaskImpl>(task_node,  context);
    }
    else if(task_type == "Com")
    {
        task_desc = std::make_shared<ComTaskImpl>(task_node,  context);
    }
    else if(task_type == "Postural")
    {
        task_desc = std::make_shared<PosturalTaskImpl>(task_node,  context);
    }
    else if(task_type == "JointLimits")
    {
        task_desc = std::make_shared<JointLimitsImpl>(task_node,  context);
    }
    else if(task_type == "JointLimitsInvariance")
    {
        task_desc = std::make_shared<JointLimitsInvarianceImpl>(task_node,  context);
    }
    else if(task_type == "VelocityLimits")
    {
        task_desc = std::make_shared<VelocityLimitsImpl>(task_node,  context);
    }
    //        else if(task_type == "Gaze")
    //        {
    //            task_desc = GazeTask::yaml_parse_gaze(task_node, model);
    //        }
    //        else if(task_type == "MinJointVel")
    //        {
    //            task_desc = MinJointVelTask::yaml_parse_minjointvel(task_node, model);
    //        }
    else
    {
        throw BadTaskDescription("Unsupported task type '" + task_type + "', maybe you forgot to specify the 'lib_name' field");
    }

    
    return task_desc;
}

BadTaskDescription::BadTaskDescription(std::string what):
    std::runtime_error(what)
{

}


TaskFactory::TaskFactory(YAML::Node prob_desc,
                         Context::ConstPtr context):
    _context(context),
    _prob_desc(prob_desc)
{

}

TaskDescription::Ptr TaskFactory::makeTask(std::string task_name)
{
    TaskDescription::Ptr task_desc;

    try
    {
        task_desc = MakeTaskDescription(_prob_desc,
                                        task_name,
                                        _context);
    }
    catch(TaskIsSubtask& e) // handle case where we got a subtask of another task
    {
        if(_subtask_map.count(e.real_task_name) == 0)
        {

            auto real_task = MakeTaskDescription(_prob_desc,
                                                 e.real_task_name,
                                                 _context);

            _subtask_map[e.real_task_name] = real_task;
        }

        auto task_impl = std::dynamic_pointer_cast<TaskDescriptionImpl>(_subtask_map.at(e.real_task_name));

        if(!task_impl)
        {
            throw std::runtime_error(fmt::format("Task '{}' type is not a TaskDescriptionImpl",
                                                 _subtask_map.at(e.real_task_name)->getName()));
        }

        if(e.remove_indices.size() > 0)
        {
            for(unsigned int i = 0; i < task_desc->getSize(); ++i)
            {
                if(std::find(e.remove_indices.begin(), e.remove_indices.end(), i) == e.remove_indices.end())
                    e.indices.push_back(i);
            }

        }

        task_desc = std::make_shared<SubtaskImpl>(task_impl,
                                                  e.indices,
                                                  task_name);
    }

    return task_desc;
}

} } 

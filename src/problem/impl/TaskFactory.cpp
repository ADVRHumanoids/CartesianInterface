#include <cartesian_interface/utils/TaskFactory.h>

#include "utils/DynamicLoading.h"

#include "Cartesian.h"
#include "Interaction.h"
#include "Com.h"
#include "Postural.h"
#include "Gaze.h"
#include "Limits.h"
#include "MinJointVel.h"

#include "problem/impl/Subtask.h"

#include "fmt/format.h"

namespace XBot { namespace Cartesian { 

std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node prob_desc,
                                                     std::string task_name,
                                                     ModelInterface::ConstPtr model
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

        if(!task_node["indices"])
        {
            throw BadTaskDescription(fmt::format("Missing field 'indices' for subtask '{}'",
                                                 task_name));
        }

        TaskIsSubtask e;
        e.real_task_name = task_node["task"].as<std::string>();
        e.indices = task_node["indices"].as<std::vector<int>>();

        throw e;
    }

    /* Get task lib name */
    std::string lib_name = "";

    if(task_node["lib_name"])
    {
        lib_name = task_node["lib_name"].as<std::string>();
    }

    /* Obtain factory name from task type */
    std::string factory_name = task_type + "TaskDescriptionFactory";
    
    /* Load task descripton from library */
    TaskDescription::Ptr task_desc;

    if(!lib_name.empty())
    {
        task_desc.reset( CallFunction<TaskDescription *>(lib_name,
                                                         "create_cartesian_interface_task_description",
                                                         task_node,
                                                         model
                                                         ) );
        
        if(!task_desc)
        {
            throw BadTaskDescription("Unable to load task description from lib '" + lib_name + "'");
        }
    }
    else
    {
        if(task_type == "Cartesian") // TBD ERROR CHECKING
        {
            task_desc = std::make_shared<CartesianTaskImpl>(task_node,  model);
        }
        //        else if(task_type == "Interaction")
        //        {
        //            task_desc = InteractionTask::yaml_parse_interaction(task_node, model);
        //        }
        else if(task_type == "Com")
        {
            task_desc = std::make_shared<ComTaskImpl>(task_node,  model);
        }
        else if(task_type == "Postural")
        {
            task_desc = std::make_shared<PosturalTaskImpl>(task_node,  model);
        }
        else if(task_type == "JointLimits")
        {
            task_desc = std::make_shared<JointLimitsImpl>(task_node,  model);
        }
        else if(task_type == "VelocityLimits")
        {
            task_desc = std::make_shared<VelocityLimitsImpl>(task_node,  model);
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
    }
    
    return task_desc;
}

BadTaskDescription::BadTaskDescription(std::string what):
    std::runtime_error(what)
{

}


TaskFactory::TaskFactory(YAML::Node prob_desc,
                         ModelInterface::ConstPtr model):
    _model(model),
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
                                        _model);
    }
    catch(TaskIsSubtask& e) // handle case where we got a subtask of another task
    {
        if(_subtask_map.count(e.real_task_name) == 0)
        {

            auto real_task = MakeTaskDescription(_prob_desc,
                                                 e.real_task_name,
                                                 _model);

            _subtask_map[e.real_task_name] = real_task;
        }

        auto task_impl = std::dynamic_pointer_cast<TaskDescriptionImpl>(_subtask_map.at(e.real_task_name));

        if(!task_impl)
        {
            throw std::runtime_error(fmt::format("Task '{}' type is not a TaskDescriptionImpl",
                                                 _subtask_map.at(e.real_task_name)->getName()));
        }

        task_desc = std::make_shared<SubtaskImpl>(task_impl,
                                                  e.indices,
                                                  task_name);
    }

    return task_desc;
}

} } 

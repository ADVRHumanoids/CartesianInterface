#include <cartesian_interface/utils/TaskFactory.h>
#include <cartesian_interface/utils/LoadObject.hpp>

#include "Cartesian.h"
#include "Interaction.h"
#include "Com.h"
#include "Postural.h"
#include "Gaze.h"
#include "Limits.h"
#include "MinJointVel.h"

namespace XBot { namespace Cartesian { 

std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node task_node, 
                                                         ModelInterface::ConstPtr model,
                                                         std::string lib_name
                                                        )
{
    TaskDescription::Ptr task_desc;
    
    /* Obtain factory name from task type */
    std::string task_type = task_node["type"].as<std::string>();
    std::string factory_name = task_type + "TaskDescriptionFactory";
    
    /* Load task descripton from library */
    if(!lib_name.empty())
    {
        task_desc = Utils::LoadObject<TaskDescription>(lib_name, 
                                                       "create_cartesian_interface_task_description",
                                                       task_node,
                                                       model
                                                       );
        
        if(!task_desc)
        {
            throw std::runtime_error("Unable to load task description from lib '" + lib_name + "'");
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
            throw std::runtime_error("Unsupported task type '" + task_type + "', maybe you forgot to specify the 'lib_name' field");
        }
    }
    
    return task_desc;
}

} } 

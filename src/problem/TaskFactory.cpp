#include <cartesian_interface/utils/TaskFactory.h>
#include <cartesian_interface/utils/LoadObject.hpp>

#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/problem/Limits.h>
#include <cartesian_interface/problem/AngularMomentum.h>

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
                                                       factory_name, 
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
            throw std::runtime_error("Unsupported task type '" + task_type + "', maybe you forgot to specify the 'lib_name' field");
        }
    }
    
    if(task_node["weight"] && task_node["weight"].IsScalar())
    {
        task_desc->weight *= task_node["weight"].as<double>();
    }
    
    if(task_node["lambda"])
    {
        task_desc->lambda = task_node["lambda"].as<double>();
    }
    
    if(task_node["lambda2"])
    {
        task_desc->lambda2 = task_node["lambda2"].as<double>();
    }
    else
    {
        task_desc->lambda2 = -1.;
    }
    
    if(task_node["indices"])
    {
        std::vector<int> indices = task_node["indices"].as<std::vector<int>>();
        task_desc = indices % task_desc;
    }
    
    if(task_node["disabled_joints"])
    {
        for(auto jnode : task_node["disabled_joints"])
        {
            std::string jstr = jnode.as<std::string>();
            
            if(!model->hasJoint(jstr))
            {
                throw std::runtime_error("Undefined joint '" + jstr + "' listed among disabled joints");
            }
            
            task_desc->disabled_joints.push_back(jstr);
        }
    }
    
    if(task_node["enabled_joints"])
    {
        
        if(task_node["disabled_joints"])
        {
            throw std::runtime_error("Cannot specify both 'enabled_joints' and 'disabled_joints'");
        }
        
        task_desc->disabled_joints = model->getEnabledJointNames();
        
        for(auto jnode : task_node["enabled_joints"])
        {
            std::string jstr = jnode.as<std::string>();
            
            if(!model->hasJoint(jstr))
            {
                throw std::runtime_error("Undefined joint '" + jstr + "' listed among enabled joints");
            }
            
            auto it = std::find(task_desc->disabled_joints.begin(), task_desc->disabled_joints.end(), jstr);
            task_desc->disabled_joints.erase(it);
        }
    }
    
    task_desc->lib_name = lib_name;
    
    return task_desc;
}

std::shared_ptr<ConstraintDescription> MakeConstraintDescription(YAML::Node constr_node, 
                                                                 ModelInterface::ConstPtr model,
                                                                 std::string lib_name
                                                                 )
{
    /* Obtain factory name from task type */
    std::string constr_type = constr_node["type"].as<std::string>();
    std::string factory_name = constr_type + "ConstraintDescriptionFactory";
    
    auto constr_desc =  Utils::LoadObject<ConstraintDescription>(lib_name, 
                                                                 factory_name, 
                                                                 constr_node, 
                                                                 model);
    constr_desc->lib_name = lib_name;
    
    return constr_desc;
}                 


} } 

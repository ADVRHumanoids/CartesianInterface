#ifndef __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__
#define __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>

#include <cstdio>
#include <dlfcn.h>

namespace XBot { namespace Cartesian { 
    
    std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node task_node, 
                                                         ModelInterface::ConstPtr model,
                                                         std::string lib_name
                                                        );
    
    std::shared_ptr<ConstraintDescription> MakeConstraintDescription(YAML::Node constr_node, 
                                                                     ModelInterface::ConstPtr model,
                                                                     std::string lib_name
                                                                    );

} } 

#endif

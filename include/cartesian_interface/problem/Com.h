#ifndef __XBOT_CARTESIAN_PROBLEM_COM_H__
#define __XBOT_CARTESIAN_PROBLEM_COM_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTask : CartesianTask {
        
        typedef std::shared_ptr<ComTask> Ptr;
        typedef std::shared_ptr<const ComTask> ConstPtr;
        
        ComTask(ModelInterface::ConstPtr model);
        
        ComTask(YAML::Node node, ModelInterface::ConstPtr model);
    };
    
} }


#endif

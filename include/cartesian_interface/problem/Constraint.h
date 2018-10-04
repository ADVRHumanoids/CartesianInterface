#ifndef __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__
#define __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Base class for the description of a constraint.
     */
    struct ConstraintDescription {
        
        TaskInterface interface;
        
        std::string type;
        
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription() = default;
        ConstraintDescription(std::string type, TaskInterface interface = TaskInterface::None);
        
        virtual ~ConstraintDescription(){}
        
    };
  
    
} }


#endif
#ifndef __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__
#define __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__

#include <cartesian_interface/problem/Task.h>
#include <string>
#include <memory>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Base class for the description of a constraint.
     */
    struct ConstraintDescription 
    {
        /**
         * @brief Constraint type
         */
        std::string type;
        
        /**
         * @brief Library where constraint factories can be found
         */
        std::string lib_name;
        
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription() = default;
        ConstraintDescription(std::string type);
        
        virtual ~ConstraintDescription(){}
        
    };
    
    
    struct ConstraintFromTask : public ConstraintDescription
    {
        
        TaskDescription::Ptr task;
        
        typedef std::shared_ptr<ConstraintFromTask> Ptr;
        typedef std::shared_ptr<const ConstraintFromTask> ConstPtr;
        
        ConstraintFromTask() = default;
        ConstraintFromTask(TaskDescription::Ptr task);
        
    };
    
    ConstraintDescription::Ptr MakeConstraintFromTask(TaskDescription::Ptr task);
  
    TaskDescription::Ptr GetTaskFromConstraint(ConstraintDescription::Ptr constr);
    
} }






#endif

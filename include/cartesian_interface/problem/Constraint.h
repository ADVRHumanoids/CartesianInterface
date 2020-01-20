#ifndef __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__
#define __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__

#include <cartesian_interface/problem/Task.h>
#include <string>
#include <memory>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Base class for the description of a constraint.
     */
    struct ConstraintDescription : public virtual TaskDescription
    {
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        static bool IsConstraint(TaskDescription::ConstPtr task);
        static Ptr AsConstraint(TaskDescription::Ptr task);
        static ConstPtr AsConstraint(TaskDescription::ConstPtr task);

    };

    struct ConstraintFromTask : public virtual ConstraintDescription
    {
        typedef std::shared_ptr<ConstraintFromTask> Ptr;
        typedef std::shared_ptr<const ConstraintFromTask> ConstPtr;
        
        virtual TaskDescription::Ptr getTask() = 0;
    };
    
    ConstraintDescription::Ptr MakeConstraintFromTask(TaskDescription::Ptr task);
  
    TaskDescription::Ptr GetTaskFromConstraint(ConstraintDescription::Ptr constr);
    
} }






#endif

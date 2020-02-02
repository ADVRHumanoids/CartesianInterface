#ifndef __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_IMPL_H__

#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {
    
    
    struct ConstraintFromTaskImpl : public virtual ConstraintFromTask,
            public TaskDescriptionImpl
    {
        
        CARTESIO_DECLARE_SMART_PTR(ConstraintFromTaskImpl)
        
        ConstraintFromTaskImpl() = default;
        ConstraintFromTaskImpl(TaskDescriptionImpl::Ptr task);

        bool validate() override;
        void update(double time, double period) override;
        void reset() override;
        void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;

        TaskDescription::Ptr getTask() override;

    private:

        TaskDescriptionImpl::Ptr _task;
    };
    
    ConstraintDescription::Ptr MakeConstraintFromTask(TaskDescription::Ptr task);
  
    TaskDescription::Ptr GetTaskFromConstraint(ConstraintDescription::Ptr constr);
    
} }






#endif

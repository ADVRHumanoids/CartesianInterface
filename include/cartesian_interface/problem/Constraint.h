#ifndef __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__
#define __XBOT_CARTESIAN_PROBLEM_CONSTRAINT_H__

#include <cartesian_interface/problem/Task.h>
#include <string>
#include <memory>

namespace XBot { namespace Cartesian {
    
    
    /**
     * @brief Base class for the description of a constraint.
     */
    struct ConstraintDescription : public TaskDescription
    {
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription(std::string type,
                              std::string name,
                              int size,
                              ModelInterface::ConstPtr model);

        ConstraintDescription(YAML::Node node,
                              ModelInterface::ConstPtr model,
                              std::string name,
                              int size);
        
        static bool IsConstraint(TaskDescription::ConstPtr task);
        static Ptr AsConstraint(TaskDescription::Ptr task);
        static ConstPtr AsConstraint(TaskDescription::ConstPtr task);

        virtual ~ConstraintDescription() override = default;

    };

    struct ConstraintFromTask : public ConstraintDescription
    {
        
        TaskDescription::Ptr task;
        
        typedef std::shared_ptr<ConstraintFromTask> Ptr;
        typedef std::shared_ptr<const ConstraintFromTask> ConstPtr;
        
        ConstraintFromTask() = default;
        ConstraintFromTask(TaskDescription::Ptr task);

        

        // TaskDescription interface
    public:
        bool validate() override;
        void update(double time, double period) override;
        void reset() override;
        void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;
    };
    
    ConstraintDescription::Ptr MakeConstraintFromTask(TaskDescription::Ptr task);
  
    TaskDescription::Ptr GetTaskFromConstraint(ConstraintDescription::Ptr constr);
    
} }






#endif

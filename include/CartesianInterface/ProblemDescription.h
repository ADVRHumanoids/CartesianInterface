#ifndef __XBOT_CARTESIAN_PROBLEM_DESC_H__
#define __XBOT_CARTESIAN_PROBLEM_DESC_H__

#include <vector>
#include <memory>
#include <list>
#include <string>
#include <Eigen/Dense>

namespace XBot { namespace Cartesian {
    
    enum class TaskType { Cartesian, Postural, Com };
    
    struct TaskDescription {
        
        TaskType type;
        Eigen::MatrixXd weight;
        std::vector<int> indices;
        double lambda;
        
        typedef std::shared_ptr<TaskDescription> Ptr;
        typedef std::shared_ptr<const TaskDescription> ConstPtr;
        
        TaskDescription() = default;
        TaskDescription(TaskType type, int size);
        
        virtual ~TaskDescription(){}
        
        
    };
    
    typedef std::vector<TaskDescription::Ptr> AggregatedTask;
    typedef std::vector<AggregatedTask> Stack;
    
    struct CartesianTask : TaskDescription {
        
        std::string base_link, distal_link;
        double orientation_gain;
        
        typedef std::shared_ptr<CartesianTask> Ptr;
        typedef std::shared_ptr<const CartesianTask> ConstPtr;
        
        CartesianTask() = default;
        CartesianTask(std::string distal_link, std::string base_link = "world");
        
        
    };
    
    CartesianTask::Ptr MakeCartesian(std::string distal_link, std::string base_link = "world");
    CartesianTask::Ptr GetAsCartesian(TaskDescription::Ptr task);
    
    enum class ConstraintType { JointLimits, VelocityLimits };
    
    struct ConstraintDescription {
        
        ConstraintType type;
        
        typedef std::shared_ptr<ConstraintDescription> Ptr;
        typedef std::shared_ptr<const ConstraintDescription> ConstPtr;
        
        ConstraintDescription() = default;
        ConstraintDescription(ConstraintType type);
        
    };
    
    ConstraintDescription::Ptr MakeJointLimits();
    ConstraintDescription::Ptr MakeVelocityLimits();
        
    class ProblemDescription {
        
    public:
        
        ProblemDescription(TaskDescription::Ptr task);
        ProblemDescription(AggregatedTask task);
        ProblemDescription(Stack stack);
        
        ProblemDescription& operator<<(ConstraintDescription::Ptr constraint);
        
        int getNumTasks() const;
        AggregatedTask getTask(int id) const;
        
        const std::vector< ConstraintDescription::Ptr >& getBounds() const;
        
    private:
        
        std::vector< std::vector<TaskDescription::Ptr> > _stack;
        std::vector< ConstraintDescription::Ptr > _bounds;
        
    };
    
    
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             TaskDescription::Ptr task_2);
    
    AggregatedTask operator+(AggregatedTask task_1, 
                             TaskDescription::Ptr task_2);
    
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             AggregatedTask task_2);
    
    AggregatedTask operator+(AggregatedTask task_1, 
                             AggregatedTask task_2);
    
    TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight, TaskDescription::Ptr task);
    
    TaskDescription::Ptr operator%(TaskDescription::Ptr task, std::vector<int> indices);
    
    Stack operator/(TaskDescription::Ptr task_1, 
                    TaskDescription::Ptr task_2);
    
    Stack operator/(TaskDescription::Ptr task_1, 
                    AggregatedTask task_2);
    
    Stack operator/(AggregatedTask task_1, 
                    TaskDescription::Ptr task_2);
    
    Stack operator/(AggregatedTask task_1, 
                    AggregatedTask task_2);
    
} }


#endif








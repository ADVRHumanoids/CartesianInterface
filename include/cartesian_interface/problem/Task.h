#ifndef __XBOT_CARTESIAN_PROBLEM_TASK_H__
#define __XBOT_CARTESIAN_PROBLEM_TASK_H__

#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Supported task interfaces
     */
    enum class TaskInterface { None, Cartesian, Postural, Interaction };
    
    
    /**
     * @brief Base class for task descriptions, which contains
     * properties that are shared among all tasks.
     */
    struct TaskDescription {
        
        /**
         * @brief Task interface that must be exposed by CartesI/O
         */
        TaskInterface interface;
        
        /**
         * @brief Task type
         */
        std::string type;
        
        /**
         * @brief Library where task factories can be found
         */
        std::string lib_name;
        
        /**
         * @brief Task weight. Inside an aggregated task, each component
         * is weighted according to this variable, that is therefore useful
         * to model soft priorities. It MUST be a positive-definite symmetric
         * matrix of size equal to the task size.
         */
        Eigen::MatrixXd weight;
        
        /**
         * @brief Vector of indices representing a subtask of the original task.
         * The resulting task size is equal to indices.size(). When modifying
         * this variable manually, the weight matrix must be changed as well to 
         * reflect the size change. Otherwise, use operator%.
         */
        std::vector<int> indices;
        
        /**
         * @brief Feedback gain on the task error. Lower values
         * make the cartesian controller less reactive.
         */
        double lambda;
        
        /**
         * @brief Feedback gain on the task velocity error. Lower values
         * make the cartesian controller less reactive. 
         * NOTE: if negative this value has not been set and should not be used!
         */ 
        double lambda2;
        
        /**
         * @brief Vector of joint names that are disabled from 
         * contributing to the task.
         */
        std::vector<std::string> disabled_joints;
        
        typedef std::shared_ptr<TaskDescription> Ptr;
        typedef std::shared_ptr<const TaskDescription> ConstPtr;
        
        TaskDescription() = default;
        TaskDescription(TaskInterface ifc, std::string type, int size);
        
        virtual ~TaskDescription(){}
        
    private:
        
        
        
    };
  
    
    /**
     * @brief Typedef for a vector of tasks representing an aggregated task.
     */
    typedef std::vector<TaskDescription::Ptr> AggregatedTask;
    
    /**
     * @brief Typedef for a vector of aggregated tasks, representing
     * a hierachical optimization problem.
     */
    typedef std::vector<AggregatedTask> Stack;
    
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             TaskDescription::Ptr task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(AggregatedTask task_1, 
                             TaskDescription::Ptr task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(TaskDescription::Ptr task_1, 
                             AggregatedTask task_2);
    
    /**
     * @brief Aggregate two tasks (or aggregated tasks)
     * @return The resulting aggregated task
     */
    AggregatedTask operator+(AggregatedTask task_1, 
                             AggregatedTask task_2);
    
    /**
     * @brief Apply a weight matrix to a task
     * 
     * @param weight Symmetric positive definite matrix that is applied on the left to 
     * the task weight
     * @param task Task that we want to modify the weight of
     * @return The same pointer that was provided as input
     */
    TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight, TaskDescription::Ptr task);
    
    /**
     * @brief Apply a subtask selection to a task. The function also 
     * adjusts the task weight matrix.
     */
    TaskDescription::Ptr operator%(std::vector<int> indices, TaskDescription::Ptr task);
    
    // TBD refactor to enable more than two levels
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

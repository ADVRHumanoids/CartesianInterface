#ifndef __XBOT_CARTESIAN_PROBLEM_TASK_H__
#define __XBOT_CARTESIAN_PROBLEM_TASK_H__

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Macro.h>

#define NOTIFY_OBSERVERS(FieldName) \
    for(auto obs_weak: _observers) \
    { \
        if(auto obs = obs_weak.lock()) obs->on##FieldName##Changed(); \
    }

namespace XBot { namespace Cartesian {

class TaskObserver
{

public:

    typedef std::weak_ptr<TaskObserver> WeakPtr;

    virtual bool onWeightChanged();
    virtual bool onActivationStateChanged();

    virtual ~TaskObserver() = default;

private:


};


/**
 * @brief Base class for task descriptions, which contains
 * properties that are shared among all tasks.
 */
class TaskDescription {

public:

    CARTESIO_DECLARE_SMART_PTR(TaskDescription)

    friend TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight,
                                          TaskDescription::Ptr task);


    friend TaskDescription::Ptr operator%(std::vector<int> indices,
                                          TaskDescription::Ptr task);

    virtual bool validate() = 0;

    virtual const std::string& getName() const = 0;

    virtual const std::string& getType() const = 0;

    virtual int getSize() const = 0;

    virtual const std::string& getLibName() const = 0;

    virtual const Eigen::MatrixXd& getWeight() const = 0;
    virtual bool setWeight(const Eigen::MatrixXd& value) = 0;

    virtual const std::vector<int>& getIndices() const = 0;
    virtual void setIndices(const std::vector<int>& value) = 0;

    virtual double getLambda() const = 0;
    virtual void setLambda(double value) = 0;

    virtual const std::vector<std::string>& getDisabledJoints() const = 0;
    virtual void setDisabledJoints(const std::vector<std::string>& value) = 0;

    virtual void update(double time, double period) = 0;
    virtual void reset() = 0;

    virtual ActivationState getActivationState() const = 0;
    virtual bool setActivationState(const ActivationState& value) = 0;

    virtual void registerObserver(TaskObserver::WeakPtr obs) = 0;

    virtual void log(MatLogger::Ptr logger,
                     bool init_logger = false,
                     int buf_size = 1e5) = 0;




    template <typename TaskDerivedType>
    static bool HasType(ConstPtr task);

    virtual ~TaskDescription() = default;

};

template<typename TaskDerivedType>
bool TaskDescription::HasType(TaskDescription::ConstPtr task)
{
    return static_cast<bool>(std::dynamic_pointer_cast<const TaskDerivedType>(task));
}


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

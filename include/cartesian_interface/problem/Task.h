#ifndef __XBOT_CARTESIAN_PROBLEM_TASK_H__
#define __XBOT_CARTESIAN_PROBLEM_TASK_H__

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/Enum.h>
#include <cartesian_interface/Macro.h>


namespace XBot { namespace Cartesian {

/**
 * @brief The TaskObserver class models a base class
 * for task observers, i.e. other classed that need
 * to be notified whenever a task property is changed.
 *
 * Usage:
 *  1) implement the interface by inheriting **virtually** from TaskObserver
 *     (e.g. MyObserver : public virtual TaskObsever)
 *  2) call TaskDescription::registerObserver(myobs)
 */
class TaskObserver :
        public std::enable_shared_from_this<TaskObserver>
{
public:

    typedef std::weak_ptr<TaskObserver> WeakPtr;

    virtual bool onWeightChanged();
    virtual bool onActivationStateChanged();

    virtual ~TaskObserver() = default;
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

    /**
     * @brief validate performs a consistency check over the task internal data
     * @return true if the test has been passed successfully, false otherwise
     */
    virtual bool validate() = 0;

    /**
     * @brief getName returns the task name, i.e. its literal identifier
     */
    virtual const std::string& getName() const = 0;

    /**
     * @brief getType returns the task type, which is used to find additional
     * modules/plugins implementing addon functionalities
     */
    virtual const std::string& getType() const = 0;

    /**
     * @brief getSize returns the task size in terms of involved degrees of freedom.
     * The task weight must respect this size, and the task indices must lie in the
     * range [0, .., size - 1]
     */
    virtual int getSize() const = 0;

    /**
     * @brief getActivationState returns an enum specifiying whether or not the
     * task is active
     */
    virtual ActivationState getActivationState() const = 0;

    /**
     * @brief setActivationState allows to enable/disable a task
     */
    virtual bool setActivationState(const ActivationState& value) = 0;

    /**
     * @brief getLibName returns the library name where task-related plugins can be found
     */
    virtual const std::string& getLibName() const = 0;

    /**
     * @brief getWeight returns the task weight, i.e. a size x size symmetric, positive
     * definite matrix
     */
    virtual const Eigen::MatrixXd& getWeight() const = 0;

    /**
     * @brief setWeight allows to set the task weight, i.e. a size x size symmetric, positive
     * definite matrix
     */
    virtual bool setWeight(const Eigen::MatrixXd& value) = 0;

    /**
     * @brief getIndices returns the indices of the active task dofs, ranging from 0 to
     * size - 1
     */
    virtual const std::vector<int>& getIndices() const = 0;

    /**
     * @brief setIndices allows to set the indices of the active task dofs, as a vector
     * of integers ranging from 0 to size - 1, without repetitions
     */
    virtual void setIndices(const std::vector<int>& value) = 0;

    /**
     * @brief getLambda returns a generic feedback gain which is used by implementations
     * to tune error feedback level. The lambda gain should respect 0 <= lam <= 1.
     */
    virtual double getLambda() const = 0;

    /**
     * @brief setLambda allows to set the generic feedback gain which is used by implementations
     * to tune error feedback level. The lambda gain should respect 0 <= lam <= 1.
     */
    virtual void setLambda(double value) = 0;


    virtual const std::vector<std::string>& getDisabledJoints() const = 0;
    virtual void setDisabledJoints(const std::vector<std::string>& value) = 0;

    /**
     * @brief update is a loop function which is called at each control iteration
     * @param time is the value of the current controller time
     * @param period is the difference between to consecutive time values
     */
    virtual void update(double time, double period) = 0;

    /**
     * @brief reset updates the task references so that they match their actual value
     */
    virtual void reset() = 0;


    /**
     * @brief registerObserver adds a TaskObserver to the list of observers
     * It keeps a weak pointer to the observer object, such that its
     * lifetime is not extended
     */
    virtual void registerObserver(TaskObserver::WeakPtr obs) = 0;

    /**
     * @brief the log function dumps a log of all relevant task quantities
     * to the provided logger.
     * @param logger
     * @param init_logger: if true, the user should only pre-allocate the logged variables
     * @param buf_size: if init_logger is true, this is the buffer size to be pre-allocated
     */
    virtual void log(MatLogger::Ptr logger,
                     bool init_logger = false,
                     int buf_size = 1e5) = 0;




    template <typename TaskDerivedType>
    /**
     * @brief HasType is an utility funcion to check if a given task has a given
     * task type
     */
    static bool HasType(ConstPtr task);

    /* Virtual default destructor */
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

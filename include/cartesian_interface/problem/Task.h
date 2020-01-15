#ifndef __XBOT_CARTESIAN_PROBLEM_TASK_H__
#define __XBOT_CARTESIAN_PROBLEM_TASK_H__

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/Enum.h>

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

    virtual bool onWeightChanged() { return true; }
    virtual bool onActivationStateChanged() { return true; }

    virtual ~TaskObserver() = default;

private:


};


/**
 * @brief Base class for task descriptions, which contains
 * properties that are shared among all tasks.
 */
class TaskDescription {

public:

    typedef std::shared_ptr<TaskDescription> Ptr;
    typedef std::shared_ptr<const TaskDescription> ConstPtr;

    typedef std::function<bool(void)> Callback;

    friend TaskDescription::Ptr operator*(Eigen::Ref<const Eigen::MatrixXd> weight,
                                          TaskDescription::Ptr task);


    friend TaskDescription::Ptr operator%(std::vector<int> indices,
                                          TaskDescription::Ptr task);

    TaskDescription(std::string type,
                    std::string name,
                    int size,
                    ModelInterface::ConstPtr model);

    TaskDescription(YAML::Node node, ModelInterface::ConstPtr model,
                    std::string name, int size);


    virtual bool validate();

    const std::string& getName() const;

    std::string getType() const;

    int getSize() const;

    const std::string& getLibName() const;

    const Eigen::MatrixXd& getWeight() const;
    bool setWeight(const Eigen::MatrixXd& value);

    const std::vector<int>& getIndices() const;
    void setIndices(const std::vector<int>& value);

    double getLambda() const;
    void setLambda(double value);

    const std::vector<std::string>& getDisabledJoints() const;
    void setDisabledJoints(const std::vector<std::string>& value);

    virtual void update(double time, double period) = 0;
    virtual void reset() = 0;

    ActivationState getActivationState() const;
    bool setActivationState(const ActivationState& value);

    void registerObserver(TaskObserver::WeakPtr obs);

    virtual void log(MatLogger::Ptr logger,
                     bool init_logger = false,
                     int buf_size = 1e5);



    virtual ~TaskDescription() = default;

    ModelInterface::ConstPtr getModel() const;

protected:

    ModelInterface::ConstPtr _model;

    void setLibName(std::string lib_name);

private:

    /**
     * @brief ctrl_mode
     */
    ActivationState activ_state;

    /**
     * @brief Task type
     */
    std::string type;

    /**
     * @brief Task name
     */
    std::string name;

    /**
     * @brief Task size
     */
    int size;

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

    std::list<TaskObserver::WeakPtr> _observers;

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

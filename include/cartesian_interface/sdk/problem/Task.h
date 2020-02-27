#ifndef __XBOT_CARTESIAN_PROBLEM_TASK_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_TASK_IMPL_H__

#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/sdk/problem/Plugin.h>

namespace XBot { namespace Cartesian {

/**
 * @brief Base class for task descriptions, which contains
 * properties that are shared among all tasks.
 */
class TaskDescriptionImpl : public virtual TaskDescription {

public:

    CARTESIO_DECLARE_SMART_PTR(TaskDescriptionImpl)

    TaskDescriptionImpl(std::string type,
                    std::string name,
                    int size,
                    ModelInterface::ConstPtr model);

    TaskDescriptionImpl(YAML::Node node, ModelInterface::ConstPtr model,
                    std::string name, int size);


    virtual bool validate() override;

    const std::string& getName() const override;

    const std::string& getType() const override;

    int getSize() const override;

    const std::string& getLibName() const override;

    virtual const Eigen::MatrixXd& getWeight() const override;
    virtual bool setWeight(const Eigen::MatrixXd& value) override;

    virtual const std::vector<int>& getIndices() const override;
    virtual void setIndices(const std::vector<int>& value) override;

    virtual double getLambda() const override;
    virtual void setLambda(double value) override;

    virtual double getLambda2() const override;
    virtual bool setLambda2(double value) override;

    virtual const std::vector<std::string>& getDisabledJoints() const override;
    virtual void setDisabledJoints(const std::vector<std::string>& value) override;

    virtual void update(double time, double period) override;
    virtual void reset() override;

    virtual ActivationState getActivationState() const override;
    virtual bool setActivationState(const ActivationState& value) override;

    void registerObserver(TaskObserver::WeakPtr obs) override;

    virtual void log(MatLogger::Ptr logger,
                     bool init_logger = false,
                     int buf_size = 1e5) override;

    ModelInterface::ConstPtr getModel() const;

    virtual ~TaskDescriptionImpl() override = default;

protected:

    ModelInterface::ConstPtr _model;

    void setLibName(std::string lib_name);

    double getTime() const;

private:

    TaskDescriptionImpl();

    /**
     * @brief ctrl_mode
     */
    ActivationState _activ_state;

    /**
     * @brief Task type
     */
    std::string _type;

    /**
     * @brief Task name
     */
    std::string _name;

    /**
     * @brief Task size
     */
    int _size;

    /**
     * @brief Library where task factories can be found
     */
    std::string _lib_name;

    /**
     * @brief Task weight. Inside an aggregated task, each component
     * is weighted according to this variable, that is therefore useful
     * to model soft priorities. It MUST be a positive-definite symmetric
     * matrix of size equal to the task size.
     */
    Eigen::MatrixXd _weight;

    /**
     * @brief Vector of indices representing a subtask of the original task.
     * The resulting task size is equal to indices.size(). When modifying
     * this variable manually, the weight matrix must be changed as well to
     * reflect the size change. Otherwise, use operator%.
     */
    std::vector<int> _indices;

    /**
     * @brief Feedback gain on the task error. Lower values
     * make the cartesian controller less reactive.
     */
    double _lambda;

    /**
     * @brief Feedback gain on the task velocity error. Lower values
     * make the cartesian controller less reactive.
     * NOTE: if negative this value has not been set and should not be used!
     */
    double _lambda2;

    /**
     * @brief Vector of joint names that are disabled from
     * contributing to the task.
     */
    std::vector<std::string> _disabled_joints;

    std::list<TaskObserver::WeakPtr> _observers;

    double _time;

};


} }


#endif

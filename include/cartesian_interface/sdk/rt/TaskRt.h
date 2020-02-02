#ifndef TASKRT_H
#define TASKRT_H

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/sdk/rt/utils/spsc_queue_ci.hpp>
#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {


class TaskRt : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(TaskRt)

    TaskRt(TaskDescription::Ptr task);

    bool validate() override;
    const std::string & getName() const override;
    const std::string & getType() const override;
    int getSize() const override;
    const std::string & getLibName() const override;
    const Eigen::MatrixXd & getWeight() const override;
    bool setWeight(const Eigen::MatrixXd & value) override;
    const std::vector<int> & getIndices() const override;
    void setIndices(const std::vector<int> & value) override;
    double getLambda() const override;
    void setLambda(double value) override;
    const std::vector<std::string> & getDisabledJoints() const override;
    void setDisabledJoints(const std::vector<std::string> & value) override;
    void update(double time, double period) override;
    void reset() override;
    ActivationState getActivationState() const override;
    bool setActivationState(const ActivationState & value) override;
    void registerObserver(TaskObserver::WeakPtr obs) override;
    void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;

    virtual void callAvailable();
    virtual void sendState(bool send = true);

    static Ptr MakeInstance(TaskDescription::Ptr task);

protected:

    static const int DEFAULT_QUEUE_SIZE = 10;

    template <typename T, int N = DEFAULT_QUEUE_SIZE>
    using LockFreeQueue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<N>>;

private:

    typedef std::function<void(TaskDescription&)> CallbackType;
    LockFreeQueue<CallbackType, 1024> _cb_queue;

    struct DataToClient
    {
        ActivationState _activ_state;
        std::string _type;
        std::string _name;
        int _size;
        std::string _lib_name;
        Eigen::MatrixXd _weight;
        std::vector<int> _indices;
        double _lambda;
        double _lambda2;
        std::vector<std::string> _disabled_joints;
    };

    LockFreeQueue<DataToClient, 1024> _to_cli_queue;
    DataToClient _cli_data, _rt_data;

    TaskDescription::Ptr _task_impl;
};



} }


#endif // TASKRT_H

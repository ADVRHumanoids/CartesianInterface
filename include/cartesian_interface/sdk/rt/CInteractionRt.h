#ifndef CINTERACTIONRT_H
#define CINTERACTIONRT_H

#include <cartesian_interface/problem/Interaction.h>
#include <cartesian_interface/sdk/rt/CartesianRt.h>

namespace XBot { namespace Cartesian {


class InteractionRt : public virtual InteractionTask,
        public CartesianRt
{

public:

    InteractionRt(InteractionTask::Ptr task);

    // TaskRt interface
public:

    void callAvailable() override;
    void sendState(bool send) override;

    // CartesianTask interface
public:

    CARTESIO_DECLARE_SMART_PTR(InteractionTask)

    const Eigen::Matrix6d& getStiffness() const override;
    const Eigen::Matrix6d& getDamping() const override;
    const Eigen::Matrix6d& getInertia() const override;
    const Eigen::Vector6d& getForceReference() const override;
    void getForceLimits(Eigen::Vector6d& fmin,
						Eigen::Vector6d& fmax) const override;

    void setStiffness(const Eigen::Matrix6d& k) override;
    void setDamping(const Eigen::Matrix6d& d) override;
    void setInertia(const Eigen::Matrix6d& m) override;
    void setForceReference(const Eigen::Vector6d& f) override;
    bool setForceLimits(const Eigen::Vector6d& fmin,
						const Eigen::Vector6d& fmax) override;

private:

    typedef std::function<void(InteractionTask&)> CallbackType;
    LockFreeQueue<CallbackType, 1024> _cb_queue;

    struct DataToClient
    {
 		Eigen::Matrix6d _stiffness;
 		Eigen::Matrix6d _damping;
		Eigen::Matrix6d _inertia;
		Eigen::Vector6d _force;
		Eigen::Vector6d _force_max;
		Eigen::Vector6d _force_min;
	};

    LockFreeQueue<DataToClient, 1024> _to_cli_queue;
    DataToClient _cli_data, _rt_data;

    InteractionTask::Ptr _task_impl;

public:
    void update(double time, double period) override;
};

} }

#endif // CINTERACTIONRT_H

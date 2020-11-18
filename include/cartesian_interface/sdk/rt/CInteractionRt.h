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

    const Impedance& getImpedance() const override;
	
    const Eigen::Vector6d& getForceReference() const override;
    void getForceLimits(Eigen::Vector6d& fmin,
						Eigen::Vector6d& fmax) const override;

    void setImpedance(const Impedance & impedance) override;
	
    void setForceReference(const Eigen::Vector6d& f) override;
    bool setForceLimits(const Eigen::Vector6d& fmin,
						const Eigen::Vector6d& fmax) override;
						
	void  abortStiffnessTransition() override;
	bool  setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points) override;
	State getStiffnessState() const override;

private:

    typedef std::function<void(InteractionTask&)> CallbackType;
    LockFreeQueue<CallbackType, 1024> _cb_queue;

    struct DataToClient
    {
 		Impedance       _impedance;

		Eigen::Vector6d _force    ;
		Eigen::Vector6d _force_max;
		Eigen::Vector6d _force_min;
		
		State _stiffness_state;
	};

    LockFreeQueue<DataToClient, 1024> _to_cli_queue;
    DataToClient _cli_data, _rt_data;

    InteractionTask::Ptr _task_impl;

public:
    void update(double time, double period) override;
};

} }

#endif // CINTERACTIONRT_H

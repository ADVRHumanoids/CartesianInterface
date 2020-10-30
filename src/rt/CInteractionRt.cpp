#include "rt/CInteractionRt.h"

using namespace XBot::Cartesian;
namespace pl = std::placeholders;

InteractionRt::InteractionRt(InteractionTask::Ptr task):
    CartesianRt(task),
    _task_impl(task)
{
    InteractionRt::sendState(false);
    
	_to_cli_queue.reset(_rt_data);
    
	InteractionRt::sendState(true);
    InteractionRt::update(0, 0);
}

void InteractionRt::callAvailable()
{
    TaskRt::callAvailable();
	
    _cb_queue.consume_all([this](CallbackType& cb)
    {
        cb(*_task_impl);
    });
}

void InteractionRt::sendState(bool send)
{
	/* 
	 * fi: it would be nice to have the same structure for getters:
	 * - returned
	 * - passed by reference
	*/
	
    CartesianRt::sendState(send);

    _rt_data._damping   = _task_impl->getDamping();
    _rt_data._stiffness = _task_impl->getStiffness();
	_rt_data._inertia   = _task_impl->getInertia();
	_rt_data._force     = _task_impl->getForceReference();
	
	_task_impl->getForceLimits(_rt_data._force_min,
							   _rt_data._force_max);
	
	_to_cli_queue.push(_rt_data);
}

const Eigen::Matrix6d& InteractionRt::getStiffness() const
{
    return _cli_data._stiffness;
}

const Eigen::Matrix6d& InteractionRt::getDamping() const
{
    return _cli_data._stiffness;
}

const Eigen::Matrix6d& InteractionRt::getInertia() const
{
    return _cli_data._inertia;
}

const Eigen::Vector6d& InteractionRt::getForceReference() const
{
    return _cli_data._force;
}

void InteractionRt::getForceLimits(Eigen::Vector6d& fmin, Eigen::Vector6d& fmax) const
{
	fmin = _cli_data._force_min;
	fmax = _cli_data._force_max;
}

void InteractionRt::setStiffness(const Eigen::Matrix6d& k)
{
    auto cb = std::bind(&InteractionTask::setStiffness, pl::_1, k);
    _cb_queue.push(cb);
}

void InteractionRt::setDamping(const Eigen::Matrix6d& d)
{
	auto cb = std::bind(&InteractionTask::setDamping, pl::_1, d);
    _cb_queue.push(cb);
}

void InteractionRt::setInertia(const Eigen::Matrix6d& m)
{
	auto cb = std::bind(&InteractionTask::setInertia, pl::_1, m);
    _cb_queue.push(cb);
}

void InteractionRt::setForceReference(const Eigen::Vector6d& f)
{
	auto cb = std::bind(&InteractionTask::setForceReference, pl::_1, f);
    _cb_queue.push(cb);
}

bool InteractionRt::setForceLimits(const Eigen::Vector6d& fmin,	const Eigen::Vector6d& fmax)
{
	/* fi: why this returns a bool? */
	
	auto cb = std::bind(&InteractionTask::setForceLimits, pl::_1, fmin, fmax);
	_cb_queue.push(cb);
	
	return true;
}


void InteractionRt::update(double time, double period)
{
    CartesianRt::update(time, period);
     while(_to_cli_queue.pop(_cli_data));
}

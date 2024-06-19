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
    CartesianRt::callAvailable();
	
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

	_rt_data._impedance       = _task_impl->getImpedance     ();
	_rt_data._force           = _task_impl->getForceReference();
	_rt_data._stiffness_state = _task_impl->getStiffnessState();
	
	_task_impl->getForceLimits(_rt_data._force_max);
	
	_rt_data._impedance_ref_link = _task_impl->getImpedanceRefLink();
	
	_to_cli_queue.push(_rt_data);
}

const Impedance& InteractionRt::getImpedance()
{
	return _cli_data._impedance;
}

const Eigen::Vector6d& InteractionRt::getForceReference() const
{
    return _cli_data._force;
}

void InteractionRt::getForceLimits(Eigen::Vector6d& fmax) const
{
	fmax = _cli_data._force_max;
}

State InteractionRt::getStiffnessState() const
{
	return _rt_data._stiffness_state;
}

bool InteractionRt::setImpedance(const Impedance& impedance)
{
	auto cb = std::bind(&InteractionTask::setImpedance, pl::_1, impedance);
    _cb_queue.push(cb);

    return true;
}

void InteractionRt::setForceReference(const Eigen::Vector6d& f)
{
	auto cb = std::bind(&InteractionTask::setForceReference, pl::_1, f);
    _cb_queue.push(cb);
}

bool InteractionRt::setForceLimits(const Eigen::Vector6d& fmax)
{
	/* fi: why this returns a bool? */
	
	auto cb = std::bind(&InteractionTask::setForceLimits, pl::_1, fmax);
	_cb_queue.push(cb);
	
	return true;
}

void InteractionRt::abortStiffnessTransition()
{
	auto cb = std::bind(&InteractionTask::abortStiffnessTransition, pl::_1);
	_cb_queue.push(cb);
}

bool InteractionRt::setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
	auto cb = std::bind(&InteractionTask::setStiffnessTransition, pl::_1, way_points);
    return _cb_queue.push(cb);
}

const std::string & InteractionRt::getImpedanceRefLink() const
{
    return _rt_data._impedance_ref_link;
}

bool InteractionRt::setImpedanceRefLink(const std::string & new_impedance_ref_link)
{
    auto cb = std::bind(&InteractionTask::setImpedanceRefLink,
                        pl::_1, new_impedance_ref_link);

    return _cb_queue.push(cb);
}

void InteractionRt::update(double time, double period)
{
    CartesianRt::update(time, period);
     while(_to_cli_queue.pop(_cli_data));
}

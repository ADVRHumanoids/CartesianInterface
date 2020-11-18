#include "problem/Interaction.h"
#include "fmt/format.h"

using namespace XBot::Cartesian;

InteractionTaskImpl::InteractionTaskImpl(YAML::Node task_node,
                                         Context::ConstPtr context):
    CartesianTaskImpl(task_node, context),
    _ref_timeout(-1),
    _state(State::Online),
    _fref(Eigen::Vector6d::Zero())
{
    std::vector<double> stiffness, damping, inertia, fmin, fmax;

    if(task_node["stiffness"])
    {
        stiffness = task_node["stiffness"].as<std::vector<double>>();
        if(stiffness.size() != 6)
        {
            throw std::runtime_error("'stiffness' field for interaction tasks requires six values");
        }
        _impedance.stiffness = Eigen::Vector6d::Map(stiffness.data()).asDiagonal();
    }
    else
    {
        throw std::runtime_error("'stiffness' field required for interaction tasks (six values)");
    }


    if(task_node["damping"])
    {
        damping = task_node["damping"].as<std::vector<double>>();
        if(damping.size() != 6)
        {
            throw std::runtime_error("'damping' field for interaction tasks requires six values");
        }
        _impedance.damping = Eigen::Vector6d::Map(damping.data()).asDiagonal();
    }
    else
    {
        throw std::runtime_error("'damping' field required for interaction tasks (six values)");
    }

    if(task_node["inertia"])
    {
        inertia = task_node["inertia"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'inertia' field for interaction tasks requires six values");
        }
        _impedance.mass = Eigen::Vector6d::Map(inertia.data()).asDiagonal();
    }
    else
    {
        _impedance.mass.setIdentity();
    }

    _fmin.setConstant(-1000.0);
    _fmax.setConstant(1000.0);
    if(task_node["force_min"])
    {
        fmin = task_node["force_min"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'force_min' field for interaction tasks requires six values");
        }
        _fmin = Eigen::Vector6d::Map(fmin.data());
    }

    if(task_node["force_max"])
    {
        fmax = task_node["force_max"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'force_max' field for interaction tasks requires six values");
        }
        _fmax = Eigen::Vector6d::Map(fmax.data());
    }

    if((_fmax - _fmin).minCoeff() < 0)
    {
        throw std::runtime_error("'force_max' must be component-wise greater-equal than 'force_min'");
    }

    if(_fmax.minCoeff() < 0 || _fmin.maxCoeff() > 0)
    {
        throw std::runtime_error("'force_max' must be >= 0 && 'force_min' must be <= 0");
    }

    if(_impedance.stiffness.minCoeff() < 0 || _impedance.damping.minCoeff() < 0 || _impedance.mass.minCoeff() < 0)
    {
        throw std::runtime_error("Negative values detected in interaction parameters");
    }

}

const Impedance & XBot::Cartesian::InteractionTaskImpl::getImpedance() const
{
	return _impedance;
}

const Eigen::Vector6d& XBot::Cartesian::InteractionTaskImpl::getForceReference() const
{
    return _fref;
}

void XBot::Cartesian::InteractionTaskImpl::getForceLimits(Eigen::Vector6d& fmin,
                                                          Eigen::Vector6d& fmax) const
{
    fmin = _fmin;
    fmax = _fmax;
}

void XBot::Cartesian::InteractionTaskImpl::setImpedance(const Impedance& impedance)
{
	if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in REACHING state \n",
                            getName().c_str());
        return;
    }
    
    _impedance = impedance;
}

void XBot::Cartesian::InteractionTaskImpl::setForceReference(const Eigen::Vector6d& f)
{
    _fref = f.cwiseMin(_fmax).cwiseMax(_fmin);
    _ref_timeout = getTime() + REF_TTL;
}

bool XBot::Cartesian::InteractionTaskImpl::setForceLimits(const Eigen::Vector6d& fmin,
                                                          const Eigen::Vector6d& fmax)
{
    if((fmax - fmin).minCoeff() < 0)
    {
        return false;
    }

    if(fmax.minCoeff() < 0 || fmin.maxCoeff() > 0)
    {
        return false;
    }

    _fmin = fmin;
    _fmax = fmax;
    _fref = _fref.cwiseMin(_fmax).cwiseMax(_fmin);

    return true;

}

bool XBot::Cartesian::InteractionTaskImpl::setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    _state = State::Reaching;
    
	_interpolator->clear();
    _interpolator->addWayPoint(getTime(), _impedance.stiffness);
    
    for(const auto& wp : way_points)
    {
        _interpolator->addWayPoint(wp, getTime());
    }
    
    _interpolator->compute();
	
    return true;
}

void XBot::Cartesian::InteractionTaskImpl::abortStiffnessTransition()
{
    if(_state == State::Reaching)
    {
        _state = State::Online;
        _interpolator->clear();
    }
}

void XBot::Cartesian::InteractionTaskImpl::update(double time, double period)
{
    CartesianTaskImpl::update(time, period);

    if(time > _ref_timeout) _fref.setZero();
	
	TaskDescriptionImpl::update(time, period);

    if (_state == State::Reaching)
    {
        _impedance.stiffness = _interpolator->evaluate(time);
		
		if (_interpolator->isTrajectoryEnded(time))
        {
            _state = State::Online;
        }
    }
}

State XBot::Cartesian::InteractionTaskImpl::getStiffnessState() const
{
    return _state;
}


void XBot::Cartesian::InteractionTaskImpl::log(MatLogger2::Ptr logger,
                                               bool init_logger,
                                               int buf_size)
{
    CartesianTaskImpl::log(logger, init_logger, buf_size);

    if(init_logger)
    {
        logger->create(getName() + "_fref", 6, 1, buf_size);
        logger->create(getName() + "_k", 6, 6, buf_size);
        logger->create(getName() + "_d", 6, 6, buf_size);
        logger->create(getName() + "_m", 6, 6, buf_size);

        return;
    }

    logger->add(getName() + "_fref", _fref               );
    logger->add(getName() + "_k",    _impedance.stiffness);
    logger->add(getName() + "_d",    _impedance.damping  );
    logger->add(getName() + "_m",    _impedance.mass     );

}


AdmittanceTaskImpl::AdmittanceTaskImpl(YAML::Node task_node,
                                       Context::ConstPtr context):
    InteractionTaskImpl(task_node, context)
{
    _dz.setZero();

    if(task_node["force_estimation_chains"])
    {
        _fest_chains = task_node["force_estimation_chains"].as<std::vector<std::string>>();
    }

    if(task_node["force_dead_zone"])
    {
        auto dz = task_node["force_dead_zone"].as<std::vector<double>>();
        if(dz.size() != 6)
        {
            throw std::runtime_error("'force_dead_zone' field for interaction tasks requires six values");
        }
        _dz = Eigen::Vector6d::Map(dz.data());

        if(_dz.minCoeff() < 0)
        {
            throw std::runtime_error("'force_dead_zone' values must be >= 0");
        }
    }
}

const Eigen::Vector6d& XBot::Cartesian::AdmittanceTaskImpl::getForceDeadzone() const
{
    return _dz;
}

const std::vector<std::string>& XBot::Cartesian::AdmittanceTaskImpl::getForceEstimationChains() const
{
    return _fest_chains;
}

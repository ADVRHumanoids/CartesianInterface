#include "Interaction.h"
#include "fmt/format.h"

using namespace XBot::Cartesian;

InteractionTaskImpl::InteractionTaskImpl(YAML::Node task_node,
                                         XBot::ModelInterface::ConstPtr model):
    CartesianTaskImpl(task_node, model)
{
    std::vector<double> stiffness, damping, inertia, fmin, fmax;

    if(task_node["stiffness"])
    {
        stiffness = task_node["stiffness"].as<std::vector<double>>();
        if(stiffness.size() != 6)
        {
            throw std::runtime_error("'stiffness' field for interaction tasks requires six values");
        }
        _k = Eigen::Vector6d::Map(stiffness.data()).asDiagonal();
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
        _d = Eigen::Vector6d::Map(damping.data()).asDiagonal();
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
        _m = Eigen::Vector6d::Map(inertia.data()).asDiagonal();
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

    if(_k.minCoeff() < 0 || _d.minCoeff() < 0 || _m.minCoeff() < 0)
    {
        throw std::runtime_error("Negative values detected in interaction parameters");
    }

}


const Eigen::Matrix6d& XBot::Cartesian::InteractionTaskImpl::getStiffness() const
{
    return _k;
}


const Eigen::Matrix6d& XBot::Cartesian::InteractionTaskImpl::getDamping() const
{
    return _d;
}

const Eigen::Matrix6d& XBot::Cartesian::InteractionTaskImpl::getInertia() const
{
    return _m;
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

void XBot::Cartesian::InteractionTaskImpl::setStiffness(const Eigen::Matrix6d& k)
{
    _k = k;
}

void XBot::Cartesian::InteractionTaskImpl::setDamping(const Eigen::Matrix6d& d)
{
    _d = d;
}

void XBot::Cartesian::InteractionTaskImpl::setInertia(const Eigen::Matrix6d& m)
{
    _m = m;
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


void XBot::Cartesian::InteractionTaskImpl::update(double time, double period)
{
    CartesianTaskImpl::update(time, period);

    if(time > _ref_timeout) _fref.setZero();
}


void XBot::Cartesian::InteractionTaskImpl::log(MatLogger::Ptr logger,
                                               bool init_logger,
                                               int buf_size)
{
    if(init_logger)
    {
        logger->createVectorVariable(getName() + "_fref", 6, 1, buf_size);
        logger->createVectorVariable(getName() + "_k", 6, 1, buf_size);
        logger->createVectorVariable(getName() + "_d", 6, 1, buf_size);
        logger->createVectorVariable(getName() + "_m", 6, 1, buf_size);

        return;
    }

    logger->add(getName() + "_fref", _fref);
    logger->add(getName() + "_k",    _k);
    logger->add(getName() + "_d",    _d);
    logger->add(getName() + "_m",    _m);

}


AdmittanceTaskImpl::AdmittanceTaskImpl(YAML::Node task_node,
                                       XBot::ModelInterface::ConstPtr model):
    InteractionTaskImpl(task_node, model)
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

#include "problem/Limits.h"

#include <fmt/format.h>

using namespace XBot::Cartesian;

JointLimitsInvarianceImpl::JointLimitsInvarianceImpl(YAML::Node yaml, Context::ConstPtr context):
    TaskDescriptionImpl("JointLimitsInvariance",
                        "JointLimitsInvariance",
                        context->model()->getJointNum(),
                        context)
{
    _joint_lims = std::make_shared<JointLimitsImpl>(yaml, context);

    _qddotmax = Eigen::VectorXd::Constant(_model->getJointNum(), 1e4);
    if(yaml["qddot_limits"])
    {
        try
        { ///if just a double is passed
            for(unsigned int i = 0; i < _qddotmax.size(); ++i)
                _qddotmax[i] = yaml["qddot_limits"].as<double>();

        }
        catch (const YAML::BadConversion& e)
        {
          try
          { ///if a map is passed
            for(auto lim : yaml["qddot_limits"])
            {
                auto jname = lim.first.as<std::string>();
                auto lim_value = lim.second.as<double>();
                int idx = _model->getDofIndex(jname);

                if(!_model->hasJoint(jname))
                {
                    throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint limits", jname));
                }
                Logger::warning("Joint qddot_max for joint '%s' to %.1f \n", jname.c_str(), lim_value);

                _qddotmax[idx] = lim_value;
            }
          }
          catch (const YAML::BadConversion& e)
          {
                throw std::invalid_argument(fmt::format("qddot_limits param supported only as double or map"));
          }

        }
    }
    else
    {
        throw std::invalid_argument(fmt::format("qddot_limits param is mandatory!"));
    }
}

Eigen::VectorXd JointLimitsInvarianceImpl::getQddotMax() const
{
    return _qddotmax;
}

Eigen::VectorXd JointLimitsImpl::getQmin() const
{
    return 0.5*(_qmin + _qmax) - 0.5*(_qmax - _qmin)*_bound_scaling;
}

Eigen::VectorXd JointLimitsImpl::getQmax() const
{
    return 0.5*(_qmin + _qmax) + 0.5*(_qmax - _qmin)*_bound_scaling;
}

JointLimitsImpl::JointLimitsImpl(Context::ConstPtr context):
    TaskDescriptionImpl("JointLimits",
                        "JointLimits",
                        context->model()->getJointNum(),
                        context),
    _bound_scaling(1.0)
{
   _model->getJointLimits(_qmin, _qmax);
}

JointLimitsImpl::JointLimitsImpl(YAML::Node yaml, Context::ConstPtr context):
    TaskDescriptionImpl(yaml,
                        context,
                        "JointLimits",
                        context->model()->getJointNum()),
    _bound_scaling(1.0)
{
    _model->getJointLimits(_qmin, _qmax);

    if(yaml["bound_scaling"])
    {
        _bound_scaling = yaml["bound_scaling"].as<double>();
    }

    if(yaml["limits"])
    {
        for(auto lim : yaml["limits"])
        {
            auto jname = lim.first.as<std::string>();
            auto lim_value = lim.second.as<std::pair<double, double>>();

            if(!_model->hasJoint(jname))
            {
                throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint limits", jname));
            }

            if(lim_value.first > lim_value.second)
            {
                throw std::invalid_argument(fmt::format("Invalid joint limit for joint '{}': "
                                                        "qmin ({}) > qmax ({})",
                                                        jname, lim_value.first, lim_value.second));
            }

            int idx = _model->getDofIndex(jname);

            if(lim_value.first < _qmin[idx] || lim_value.second > _qmax[idx])
            {
                Logger::warning("Joint limit for joint '%s' [%.1f, %.1f] extend URDF limits [%.1f, %.1f] \n",
                                jname.c_str(), lim_value.first, lim_value.second,
                                _qmin[idx], _qmax[idx]);
            }

            _qmin[idx] = lim_value.first;
            _qmax[idx] = lim_value.second;

        }
    }

}

bool JointLimitsImpl::setBoundScaling(double value)
{
    _bound_scaling = value;
    return true;
}

double JointLimitsImpl::getBoundScaling() const
{
    return _bound_scaling;
}


bool XBot::Cartesian::JointLimitsImpl::validate()
{
    return (_qmax - _qmin).minCoeff() >= 0;
}

void XBot::Cartesian::JointLimitsImpl::update(double time, double period)
{
    TaskDescriptionImpl::update(time, period);
}

void XBot::Cartesian::JointLimitsImpl::reset()
{
}

VelocityLimitsImpl::VelocityLimitsImpl(Context::ConstPtr context):
    TaskDescriptionImpl("VelocityLimits",
                        "VelocityLimits",
                        context->model()->getJointNum(),
                        context),
    _bound_scaling(1.0)
{
    _model->getVelocityLimits(_qdot_max);
}

VelocityLimitsImpl::VelocityLimitsImpl(YAML::Node yaml,
                                       Context::ConstPtr context):
    TaskDescriptionImpl(yaml,
                        context,
                        "VelocityLimits",
                        context->model()->getJointNum()),
    _bound_scaling(1.0)
{
    _model->getVelocityLimits(_qdot_max);

    if(yaml["bound_scaling"])
    {
        _bound_scaling = yaml["bound_scaling"].as<double>();
    }

    if(yaml["limits"] && yaml["limits"].IsMap())
    {
        for(auto lim : yaml["limits"])
        {
            auto jname = lim.first.as<std::string>();
            auto lim_value = lim.second.as<double>();

            if(!_model->hasJoint(jname))
            {
                throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint limits", jname));
            }

            if(lim_value < 0)
            {
                throw std::invalid_argument(fmt::format("Invalid velocity limit for joint '{}': "
                                                        "qdot_max ({}) < 0",
                                                        jname, lim_value));
            }

            int idx = _model->getDofIndex(jname);

            if(lim_value > _qdot_max[idx])
            {
                Logger::warning("Velocity limit for joint '%s' [%.1f] extends URDF limits [%.1f] \n",
                                jname.c_str(), lim_value,
                                _qdot_max[idx]);
            }

            _qdot_max[idx] = lim_value;

        }
    }

    if(yaml["limits"] && yaml["limits"].IsScalar())
    {
        _qdot_max.setConstant(_model->getJointNum(), yaml["limits"].as<double>());
    }

}

Eigen::VectorXd VelocityLimitsImpl::getQdotMax() const
{
    return _bound_scaling * _qdot_max;
}

bool VelocityLimitsImpl::validate()
{
    return _qdot_max.minCoeff() >= 0.0 && _bound_scaling >= 0.0;
}

void VelocityLimitsImpl::update(double time, double period)
{
    TaskDescriptionImpl::update(time, period);
}

void VelocityLimitsImpl::reset()
{

}

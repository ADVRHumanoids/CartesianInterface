#include <cartesian_interface/problem/Limits.h>

#include <fmt/format.h>

using namespace XBot::Cartesian;

Eigen::VectorXd JointLimits::getQmin() const
{
    return _qmin;
}

Eigen::VectorXd JointLimits::getQmax() const
{
    return _qmax;
}

JointLimits::JointLimits(XBot::ModelInterface::ConstPtr model):
    ConstraintDescription("JointLimits",
                          "JointLimits",
                          model->getJointNum(),
                          model),
    _bound_scaling(1.0)
{
    model->getJointLimits(_qmin, _qmax);
}

JointLimits::JointLimits(YAML::Node yaml, XBot::ModelInterface::ConstPtr model):
    ConstraintDescription(yaml,
                          model,
                          "JointLimits",
                          model->getJointNum()),
    _bound_scaling(1.0)
{
     model->getJointLimits(_qmin, _qmax);

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

            if(!model->hasJoint(jname))
            {
                throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint limits", jname));
            }

            if(lim_value.first > lim_value.second)
            {
                throw std::invalid_argument(fmt::format("Invalid joint limit for joint '{}': "
                                                        "qmin ({}) > qmax ({})",
                                                        jname, lim_value.first, lim_value.second));
            }

            int idx = model->getDofIndex(jname);

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

bool JointLimits::setBoundScaling(double value)
{
    _bound_scaling = value;
    return true;
}

double JointLimits::getBoundScaling() const
{
    return _bound_scaling;
}


bool XBot::Cartesian::JointLimits::validate()
{
    return (_qmax - _qmin).minCoeff() >= 0;
}

void XBot::Cartesian::JointLimits::update(double time, double period)
{
    TaskDescription::update(time, period);
}

void XBot::Cartesian::JointLimits::reset()
{
}

VelocityLimits::VelocityLimits(XBot::ModelInterface::ConstPtr model):
    ConstraintDescription("VelocityLimits",
                          "VelocityLimits",
                          model->getJointNum(),
                          model),
    _bound_scaling(1.0)
{
    model->getVelocityLimits(_qdot_max);
}

VelocityLimits::VelocityLimits(YAML::Node yaml,
                               XBot::ModelInterface::ConstPtr model):
    ConstraintDescription(yaml,
                          model,
                          "VelocityLimits",
                          model->getJointNum()),
    _bound_scaling(1.0)
{
     model->getVelocityLimits(_qdot_max);

     if(yaml["bound_scaling"])
     {
         _bound_scaling = yaml["bound_scaling"].as<double>();
     }

     if(yaml["limits"])
     {
        for(auto lim : yaml["limits"])
        {
            auto jname = lim.first.as<std::string>();
            auto lim_value = lim.second.as<double>();

            if(!model->hasJoint(jname))
            {
                throw std::invalid_argument(fmt::format("Invalid joint '{}' in joint limits", jname));
            }

            if(lim_value < 0)
            {
                throw std::invalid_argument(fmt::format("Invalid velocity limit for joint '{}': "
                                                        "qdot_max ({}) < 0",
                                                        jname, lim_value));
            }

            int idx = model->getDofIndex(jname);

            if(lim_value > _qdot_max[idx])
            {
                Logger::warning("Velocity limit for joint '%s' [%.1f] extends URDF limits [%.1f] \n",
                                jname.c_str(), lim_value,
                                _qdot_max[idx]);
            }

            _qdot_max[idx] = lim_value;

        }
     }

}

Eigen::VectorXd VelocityLimits::getQdotMax() const
{
    return _bound_scaling * _qdot_max;
}

bool VelocityLimits::validate()
{
    return _qdot_max.minCoeff() > 0.0 && _bound_scaling > 0.0;
}

void VelocityLimits::update(double time, double period)
{
    ConstraintDescription::update(time, period);
}

void VelocityLimits::reset()
{

}

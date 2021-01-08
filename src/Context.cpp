#include <cartesian_interface/Context.h>
#include <iostream>

using namespace XBot::Cartesian;

Parameters::Parameters(double dt,
                       std::string log_path):
    _dt(dt),
    _log_path(log_path),
    _enable_log(true)
{

}

bool Parameters::isLogEnabled() const
{
    return _enable_log;
}

void Parameters::setLogEnabled(bool flag)
{
    _enable_log = flag;
}

double Parameters::getControlPeriod() const
{
    return _dt;
}

std::string Parameters::getLogPath() const
{
    return _log_path;
}

Context::Context(Parameters::Ptr params,
                 ModelInterface::Ptr model):
    _params(params),
    _model(model)
{
    if(!model)
    {
        throw std::invalid_argument("Model is nullptr");
    }
}

Parameters::Ptr Context::params()
{
    return _params;
}

Parameters::ConstPtr Context::params() const
{
    return _params;
}

XBot::ModelInterface::Ptr Context::model()
{
    return _model;
}

XBot::ModelInterface::ConstPtr Context::model() const
{
    return _model;
}

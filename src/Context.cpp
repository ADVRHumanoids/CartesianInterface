#include <cartesian_interface/Context.h>
#include <iostream>

using namespace XBot::Cartesian;



Parameters::Parameters(double dt):
    _dt(dt)
{

}

double Parameters::getControlPeriod() const
{
    return _dt;
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

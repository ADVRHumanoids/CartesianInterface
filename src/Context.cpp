#include <cartesian_interface/Context.h>
#include <iostream>

using namespace XBot::Cartesian;

class XBot::Cartesian::ContextImpl
{

public:

    double control_period;
};

std::weak_ptr<ContextImpl> Context::_weak_impl;

Context::Context()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    if(auto ptr = _weak_impl.lock())
    {
        _impl = ptr;
    }
    else
    {
        throw ContextEmpty();
    }
}

Context::Context(double control_period):
    _control_period(control_period)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

double Context::getControlPeriod() const
{
    return _impl->control_period;
}

Context::~Context()
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
}

Context::Ptr Context::MakeContext(double control_period)
{
    if(auto ptr = _weak_impl.lock())
    {
        throw ContextRedefinition();
    }

    auto instance = std::make_shared<ContextImpl>();
    instance->control_period = control_period;

    _weak_impl = instance;

    return std::make_shared<Context>();
}

ContextInvalid::ContextInvalid(std::string reason):
    _reason(reason)
{

}

const char * ContextInvalid::what() const noexcept
{
    return ("Invalid context object: " + _reason).c_str();
}

const char * ContextRedefinition::what() const noexcept
{
    return "Valid context already defined";
}

const char * ContextEmpty::what() const noexcept
{
    return "Context is undefined";
}


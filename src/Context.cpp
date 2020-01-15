#include <cartesian_interface/Context.h>

using namespace XBot::Cartesian;

struct XBot::Cartesian::ContextImpl
{
    double control_period;

    ContextImpl();
};

std::weak_ptr<ContextImpl> Context::_weak_impl;

Context::Context()
{
    if(auto ptr = _weak_impl.lock())
    {
        _impl = ptr;
    }
    else
    {
        throw ContextEmpty();
    }
}

Context::Context(std::shared_ptr<ContextImpl> ctx)
{
    if(!ctx)
    {
        throw ContextInvalid("provided impl is nullptr");
    }

    if(auto ptr = _weak_impl.lock())
    {
        throw ContextRedefinition();
    }

    _impl = ctx;
    _weak_impl = ctx;
}

double Context::getControlPeriod() const
{
    return _impl->control_period;
}

Context::~Context()
{

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

ContextImpl::ContextImpl():
    control_period(0.01)
{

}

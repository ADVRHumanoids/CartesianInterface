#include "RosContext.h"

#include <cartesian_interface/Context.h>

using namespace XBot::Cartesian;

struct XBot::Cartesian::RosContextImpl
{
    ros::NodeHandle nh;
    std::string tf_prefix;
    std::string tf_prefix_slash;

    RosContextImpl();
};

std::weak_ptr<RosContextImpl> RosContext::_weak_impl;

RosContext::RosContext()
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

RosContext::RosContext(std::shared_ptr<RosContextImpl> ctx)
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

ros::NodeHandle & RosContext::nh()
{
    return _impl->nh;
}

const std::string & RosContext::tf_prefix() const
{
   return _impl->tf_prefix;
}

const std::string & RosContext::tf_prefix_slash() const
{
    return _impl->tf_prefix_slash;
}

RosContextImpl::RosContextImpl()
{

}

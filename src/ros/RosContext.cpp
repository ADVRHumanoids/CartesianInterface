#include "RosContext.h"

#include <cartesian_interface/Context.h>

using namespace XBot::Cartesian;

class XBot::Cartesian::RosContextImpl
{

public:

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

RosContext::Ptr RosContext::MakeContext(ros::NodeHandle nh,
                                        std::string tf_prefix)
{
    if(auto ptr = _weak_impl.lock())
    {
        throw ContextRedefinition();
    }

    auto instance = std::make_shared<RosContextImpl>();
    instance->nh = nh;
    instance->tf_prefix = tf_prefix;
    instance->tf_prefix_slash = tf_prefix == "" ? "" : (tf_prefix + "/");

    _weak_impl = instance;

    return std::make_shared<RosContext>();
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

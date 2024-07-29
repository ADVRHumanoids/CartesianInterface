#include "ros/RosContext.h"

#include <cartesian_interface/Context.h>

using namespace XBot::Cartesian;


RosContext::RosContext(rclcpp::Node::SharedPtr node,
                       std::string tf_prefix,
                       Context::ConstPtr ci_context):
    _node(node),
    _tf_prefix(tf_prefix),
    _tf_prefix_slash(tf_prefix == "" ? "" : (tf_prefix + "/")),
    _ci_ctx(ci_context)
{
}


rclcpp::Node::SharedPtr RosContext::node()
{
    return _node;
}

const std::string & RosContext::tf_prefix() const
{
   return _tf_prefix;
}

const std::string & RosContext::tf_prefix_slash() const
{
    return _tf_prefix_slash;
}

Context::ConstPtr RosContext::ci_context() const
{
    return _ci_ctx;
}


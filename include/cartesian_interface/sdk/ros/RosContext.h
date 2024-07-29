#ifndef ROSCONTEXT_H
#define ROSCONTEXT_H

#include <rclcpp/rclcpp.hpp>
#include <cartesian_interface/Context.h>

namespace XBot { namespace Cartesian {

class RosContext
{

public:

    CARTESIO_DECLARE_SMART_PTR(RosContext)

    RosContext(rclcpp::Node::SharedPtr node,
               std::string tf_prefix,
               Context::ConstPtr ci_context);

    rclcpp::Node::SharedPtr node();
    const std::string& tf_prefix() const;
    const std::string& tf_prefix_slash() const;
    Context::ConstPtr ci_context() const;

private:

    rclcpp::Node::SharedPtr _node;
    std::string _tf_prefix;
    std::string _tf_prefix_slash;
    Context::ConstPtr _ci_ctx;

};

} }

#endif // ROSCONTEXT_H

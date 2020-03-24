#ifndef ROSCONTEXT_H
#define ROSCONTEXT_H

#include <ros/ros.h>
#include <cartesian_interface/Context.h>

namespace XBot { namespace Cartesian {

class RosContext
{

public:

    CARTESIO_DECLARE_SMART_PTR(RosContext)

    RosContext(ros::NodeHandle nh,
               std::string tf_prefix,
               Context::ConstPtr ci_context);

    ros::NodeHandle& nh();
    const std::string& tf_prefix() const;
    const std::string& tf_prefix_slash() const;
    Context::ConstPtr ci_context() const;

private:

    ros::NodeHandle _nh;
    std::string _tf_prefix;
    std::string _tf_prefix_slash;
    Context::ConstPtr _ci_ctx;

};

} }

#endif // ROSCONTEXT_H

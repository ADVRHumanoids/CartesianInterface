#ifndef ROSCONTEXT_H
#define ROSCONTEXT_H

#include <ros/ros.h>
#include <cartesian_interface/Context.h>

namespace XBot { namespace Cartesian {

class RosContextImpl;

class RosContext
{

public:

    CARTESIO_DECLARE_SMART_PTR(RosContext)

    RosContext();

    ros::NodeHandle& nh();
    const std::string& tf_prefix() const;
    const std::string& tf_prefix_slash() const;

    static Ptr MakeContext(ros::NodeHandle nh,
                           std::string tf_prefix);

private:

    static std::weak_ptr<RosContextImpl> _weak_impl;

    std::shared_ptr<RosContextImpl> _impl;

};

} }

#endif // ROSCONTEXT_H

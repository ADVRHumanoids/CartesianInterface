#ifndef PLUGIN_ROS_H
#define PLUGIN_ROS_H

#include <cartesian_interface/Macro.h>

#define CARTESIO_REGISTER_ROS_API_PLUGIN(ClassName, TaskType) \
extern "C" ::XBot::Cartesian::ServerApi::TaskRos * \
    create_cartesio_##TaskType##_ros_api(::XBot::Cartesian::TaskDescription::Ptr task, \
                                ::XBot::Cartesian::RosContext::Ptr context, \
                                ::XBot::Cartesian::detail::Version ci_ver) \
{ \
    ::XBot::Cartesian::detail::Version plugin_ver CARTESIO_ABI_VERSION; \
\
    if(!plugin_ver.isCompatible(ci_ver)) \
    { \
        fprintf(stderr,  \
                "Incompatible plugin version %d.%d.%d (expected %d.%d.%d)", \
                plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                ci_ver.major, ci_ver.minor, ci_ver.patch); \
        throw std::runtime_error("Version mismatch"); \
    } \
\
    if(!(ci_ver == plugin_ver)) \
    { \
        fprintf(stderr,  \
                "Different plugin version %d.%d.%d (expected %d.%d.%d)", \
                plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                ci_ver.major, ci_ver.minor, ci_ver.patch); \
    } \
\
    return new ClassName(task, context); \
}


#define CARTESIO_REGISTER_ROS_CLIENT_API_PLUGIN(ClassName, TaskType) \
extern "C" ::XBot::Cartesian::ClientApi::TaskRos * \
    create_cartesio_##TaskType##_ros_client_api(::std::string name, \
                                         ::ros::NodeHandle nh, \
                                         ::XBot::Cartesian::detail::Version ci_ver) \
{ \
    ::XBot::Cartesian::detail::Version plugin_ver CARTESIO_ABI_VERSION; \
\
    if(!plugin_ver.isCompatible(ci_ver)) \
    { \
        fprintf(stderr,  \
                "Incompatible plugin version %d.%d.%d (expected %d.%d.%d)", \
                plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                ci_ver.major, ci_ver.minor, ci_ver.patch); \
        throw std::runtime_error("Version mismatch"); \
    } \
\
    if(!(ci_ver == plugin_ver)) \
    { \
        fprintf(stderr,  \
                "Different plugin version %d.%d.%d (expected %d.%d.%d)", \
                plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                ci_ver.major, ci_ver.minor, ci_ver.patch); \
    } \
\
    return new ClassName(name, nh); \
}


#endif // PLUGIN_H

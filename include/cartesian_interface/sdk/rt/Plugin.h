#ifndef PLUGIN_RT_H
#define PLUGIN_RT_H

#include <cartesian_interface/Macro.h>

#define CARTESIO_REGISTER_RT_API_PLUGIN(ClassName, TaskType) \
extern "C" ::XBot::Cartesian::TaskRt * \
    create_cartesio_##TaskType##_rt_api(::XBot::Cartesian::TaskDescription::Ptr task, \
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
    return new ClassName(task); \
}

#endif // PLUGIN_RT_H

#ifndef PLUGIN_TASK_H
#define PLUGIN_TASK_H

// create_cartesian_interface_task_description
#define CARTESIO_REGISTER_TASK_PLUGIN(ClassName, TaskType) \
extern "C" ::XBot::Cartesian::TaskDescription* create_cartesio_##TaskType##_description(YAML::Node task_node, \
                                                                     ::XBot::ModelInterface::ConstPtr model, \
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
    return new ClassName(task_node, model); \
}

#endif // PLUGIN_H

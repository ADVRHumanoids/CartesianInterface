#ifndef PLUGIN_H
#define PLUGIN_H

#define CARTESIO_REGISTER_OPENSOT_PLUGIN(FactoryName, ArgType, ClassName, RetType) \
extern "C" RetType* FactoryName(::XBot::Cartesian::ArgType::Ptr task, \
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
    return new ClassName(task, model); \
}

#define CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(ClassName, TaskType) \
    CARTESIO_REGISTER_OPENSOT_PLUGIN(create_opensot_##TaskType##_adapter, \
                                     TaskDescription, \
                                     ClassName, \
                                     ::XBot::Cartesian::OpenSotTaskAdapter)

#define CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(ClassName, TaskType) \
    CARTESIO_REGISTER_OPENSOT_PLUGIN(create_opensot_##TaskType##_adapter, \
                                     ConstraintDescription, \
                                     ClassName,  \
                                     ::XBot::Cartesian::OpenSotConstraintAdapter)

#endif // PLUGIN_H

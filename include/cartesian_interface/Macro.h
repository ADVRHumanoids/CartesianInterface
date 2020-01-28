#ifndef MACRO_H
#define MACRO_H

namespace XBot { namespace Cartesian { namespace detail {

struct Version
{
    int major;
    int minor;
    int patch;

    bool isCompatible(Version other)
    {
        return major == other.major && minor == other.minor;
    }
};

inline bool operator==(Version a, Version b)
{
    return a.major == b.major &&
            a.minor == b.minor &&
            a.patch == b.patch;
}

} } }

#define CARTESIO_VERSION {2, 0, 0}

#define CARTESIO_DECLARE_SMART_PTR(Class) \
    typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr; \

#define NOTIFY_OBSERVERS(FieldName) \
    bool notify_success = true; \
    for(auto obs_weak: _observers) \
    { \
        if(auto obs = obs_weak.lock()) \
        {   \
            if(!obs->on##FieldName##Changed()) notify_success = false; \
        } \
    }
// create_cartesian_interface_task_description
#define CARTESIO_REGISTER_TASK_PLUGIN(TaskType) \
extern "C" TaskDescription* create_cartesian_interface_task_description(YAML::Node task_node, \
                                                                        ::XBot::ModelInterface::ConstPtr model, \
                                                                        ::XBot::Cartesian::detail::Version ci_ver) \
{ \
    ::XBot::Cartesian::detail::Version plugin_ver = CARTESIO_VERSION; \
\
    if(!plugin_ver.isCompatible(ci_ver)) \
    { \
        throw std::runtime_error(fmt::format("Incompatible plugin version {}.{}.{} (expected {}.{}.{})", \
                                             plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                                             ci_ver.major, ci_ver.minor, ci_ver.patch)); \
    } \
\
    if(!(ci_ver == plugin_ver)) \
    { \
        fmt::print("Warning: plugin version {}.{}.{} does not match expected {}.{}.{}", \
                    plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                    ci_ver.major, ci_ver.minor, ci_ver.patch); \
    } \
\
    return new TaskType(task_node, model); \
}

#define CARTESIO_REGISTER_OPENSOT_PLUGIN(FactoryName, TaskType, RetType) \
extern "C" RetType* FactoryName(::XBot::Cartesian::TaskDescription::Ptr task, \
                                ::XBot::ModelInterface::ConstPtr model, \
                                ::XBot::Cartesian::detail::Version ci_ver) \
{ \
    ::XBot::Cartesian::detail::Version plugin_ver = CARTESIO_VERSION; \
\
    if(!plugin_ver.isCompatible(ci_ver)) \
    { \
        throw std::runtime_error(fmt::format("Incompatible plugin version {}.{}.{} (expected {}.{}.{})", \
                                             plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                                             ci_ver.major, ci_ver.minor, ci_ver.patch)); \
    } \
\
    if(!(ci_ver == plugin_ver)) \
    { \
        fmt::print("Warning: plugin version {}.{}.{} does not match expected {}.{}.{}", \
                    plugin_ver.major, plugin_ver.minor, plugin_ver.patch, \
                    ci_ver.major, ci_ver.minor, ci_ver.patch); \
    } \
\
    return new TaskType(task, model); \
}

#define CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(TaskType) \
    CARTESIO_REGISTER_OPENSOT_PLUGIN(create_opensot_task_adapter, \
                                     TaskType, \
                                     ::XBot::Cartesian::OpenSotTaskAdapter)

#define CARTESIO_REGISTER_OPENSOT_CONSTR_PLUGIN(TaskType) \
    CARTESIO_REGISTER_OPENSOT_PLUGIN(create_opensot_constr_adapter, \
                                     TaskType,  \
                                     ::XBot::Cartesian::OpenSotConstraintAdapter)

#define CARTESIO_REGISTER_ROS_API_PLUGIN(TaskType) \
    CARTESIO_REGISTER_OPENSOT_PLUGIN(create_cartesio_ros_api, \
                                     TaskType,  \
                                     ::XBot::Cartesian::ServerApi::TaskRos)

#endif // MACRO_H

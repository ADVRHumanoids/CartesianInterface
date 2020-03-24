#ifndef SOLVERPLUGIN_H
#define SOLVERPLUGIN_H

#define CARTESIO_REGISTER_SOLVER_PLUGIN(ClassName, SolverName) \
extern "C" ::XBot::Cartesian::CartesianInterfaceImpl* \
    create_cartesio_##SolverName##_solver(::XBot::Cartesian::ProblemDescription ik_pb, \
                                ::XBot::Cartesian::Context::Ptr context, \
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
    return new ClassName(ik_pb, context); \
}


#endif // SOLVERPLUGIN_H

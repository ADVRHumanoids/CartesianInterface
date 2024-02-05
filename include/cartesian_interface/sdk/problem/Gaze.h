#ifndef __XBOT_CARTESIAN_PROBLEM_GAZE_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_GAZE_IMPL_H__

#include <cartesian_interface/problem/Gaze.h>
#include <cartesian_interface/sdk/problem/Cartesian.h>

namespace XBot { namespace Cartesian {

    /**
     * @brief Description of a center of mass task
     */
    struct GazeTaskImpl : public CartesianTaskImpl,
                         public virtual GazeTask
    {

        CARTESIO_DECLARE_SMART_PTR(GazeTaskImpl)


        GazeTaskImpl(Context::ConstPtr context,
                          std::string name,
                          std::string distal_link,
                          std::string base_link = "world");

        GazeTaskImpl(YAML::Node node, Context::ConstPtr context);
    };

} }


#endif

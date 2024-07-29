#ifndef __XBOT_CARTESIAN_PROBLEM_GAZE_H__
#define __XBOT_CARTESIAN_PROBLEM_GAZE_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a Gaze task
     */
    struct GazeTask : virtual CartesianTask {

        typedef std::shared_ptr<GazeTask> Ptr;
        typedef std::shared_ptr<const GazeTask> ConstPtr;
    };
  
    
} }


#endif

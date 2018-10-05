#ifndef __XBOT_CARTESIAN_PROBLEM_GAZE_H__
#define __XBOT_CARTESIAN_PROBLEM_GAZE_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a Gaze task
     */
    struct GazeTask : CartesianTask {

        typedef std::shared_ptr<GazeTask> Ptr;
        typedef std::shared_ptr<const GazeTask> ConstPtr;

        GazeTask() = default;
        GazeTask(std::string base_link = "world");
    };

    /**
     * @brief Make a Gaze task and return a shared pointer
     */
    GazeTask::Ptr MakeGaze(std::string base_link = "world");

    /**
     * @brief Dynamic cast a generic task to a GazeTask
     *
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a GazeTask)
     */
    GazeTask::Ptr GetAsGaze(TaskDescription::Ptr task);
  
    
} }


#endif
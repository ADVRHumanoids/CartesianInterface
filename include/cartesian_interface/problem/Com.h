#ifndef __XBOT_CARTESIAN_PROBLEM_COM_H__
#define __XBOT_CARTESIAN_PROBLEM_COM_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTask : CartesianTask {
        
        typedef std::shared_ptr<ComTask> Ptr;
        typedef std::shared_ptr<const ComTask> ConstPtr;
        
        ComTask();
        
    };
    
    /**
     * @brief Make a CoM task and return a shared pointer
     */
    ComTask::Ptr MakeCom();
    
    /**
     * @brief Dynamic cast a generic task to a ComTask
     * 
     * @return A null pointer if the cast is unsuccessful (i.e. task is not a ComTask)
     */
    ComTask::Ptr GetAsCom(TaskDescription::Ptr task);
  
    
} }


#endif
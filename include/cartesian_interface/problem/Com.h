#ifndef __XBOT_CARTESIAN_PROBLEM_COM_H__
#define __XBOT_CARTESIAN_PROBLEM_COM_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTask : virtual CartesianTask {
        
        typedef std::shared_ptr<ComTask> Ptr;
        typedef std::shared_ptr<const ComTask> ConstPtr;
    };
    
} }


#endif

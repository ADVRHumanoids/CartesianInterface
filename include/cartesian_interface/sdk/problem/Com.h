#ifndef __XBOT_CARTESIAN_PROBLEM_COM_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_COM_IMPL_H__

#include <cartesian_interface/problem/Com.h>
#include <cartesian_interface/sdk/problem/Cartesian.h>

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTaskImpl : public CartesianTaskImpl,
                         public virtual ComTask
    {
        
        CARTESIO_DECLARE_SMART_PTR(ComTaskImpl)
        
        ComTaskImpl(Context::ConstPtr context);
        
        ComTaskImpl(YAML::Node node, Context::ConstPtr context);
    };
    
} }


#endif

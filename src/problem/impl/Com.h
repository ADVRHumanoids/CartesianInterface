#ifndef __XBOT_CARTESIAN_PROBLEM_COM_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_COM_IMPL_H__

#include <cartesian_interface/problem/Com.h>
#include "Cartesian.h"

namespace XBot { namespace Cartesian {
    
    /**
     * @brief Description of a center of mass task
     */
    struct ComTaskImpl : public CartesianTaskImpl,
                         public virtual ComTask
    {
        
        CARTESIO_DECLARE_SMART_PTR(ComTaskImpl)
        
        ComTaskImpl(ModelInterface::ConstPtr model);
        
        ComTaskImpl(YAML::Node node, ModelInterface::ConstPtr model);
    };
    
} }


#endif

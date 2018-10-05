#ifndef __XBOT_CARTESIAN_PROBLEM_LIMITS_H__
#define __XBOT_CARTESIAN_PROBLEM_LIMITS_H__

#include <cartesian_interface/problem/Constraint.h>

namespace XBot { namespace Cartesian {
    
    /**
    * @brief Construct a joint limits constraint and return a shared pointer
    */
    ConstraintDescription::Ptr MakeJointLimits();
    
    /**
    * @brief Construct a joint velocity limits constraint and return a shared pointer
    */
    ConstraintDescription::Ptr MakeVelocityLimits();
  
    
} }


#endif
#ifndef __XBOT_CARTESIAN_PROBLEM_LIMITS_H__
#define __XBOT_CARTESIAN_PROBLEM_LIMITS_H__

#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/Macro.h>

namespace XBot { namespace Cartesian {
    
    class JointLimits : public virtual ConstraintDescription
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(JointLimits)

        virtual bool setBoundScaling(double value) = 0;
        virtual double getBoundScaling() const = 0;

        virtual Eigen::VectorXd getQmin() const = 0;
        virtual Eigen::VectorXd getQmax() const = 0;

    };

    class VelocityLimits : public virtual ConstraintDescription
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(VelocityLimits)

        virtual Eigen::VectorXd getQdotMax() const = 0;

    };
    
} }


#endif

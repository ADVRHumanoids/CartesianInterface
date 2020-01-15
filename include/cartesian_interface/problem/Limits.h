#ifndef __XBOT_CARTESIAN_PROBLEM_LIMITS_H__
#define __XBOT_CARTESIAN_PROBLEM_LIMITS_H__

#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/Macro.h>

namespace XBot { namespace Cartesian {
    
    class JointLimits : public ConstraintDescription
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(JointLimits)

        JointLimits(ModelInterface::ConstPtr model);
        JointLimits(YAML::Node yaml, ModelInterface::ConstPtr model);

        bool setBoundScaling();
        double getBoundScaling() const;

        Eigen::VectorXd getQmin() const;
        Eigen::VectorXd getQmax() const;

    private:

        double _bound_scaling;
        Eigen::VectorXd _qmin, _qmax;
    };
    
} }


#endif

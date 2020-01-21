#ifndef __XBOT_CARTESIAN_PROBLEM_LIMITS_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_LIMITS_IMPL_H__

#include <cartesian_interface/problem/Limits.h>
#include "Constraint.h"

namespace XBot { namespace Cartesian {
    
    class JointLimitsImpl : public virtual JointLimits,
            public TaskDescriptionImpl
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(JointLimitsImpl)

        JointLimitsImpl(ModelInterface::ConstPtr model);
        JointLimitsImpl(YAML::Node yaml, ModelInterface::ConstPtr model);

        bool setBoundScaling(double value) override;
        double getBoundScaling() const override;

        Eigen::VectorXd getQmin() const override;
        Eigen::VectorXd getQmax() const override;

    private:

        double _bound_scaling;
        Eigen::VectorXd _qmin, _qmax;

        // TaskDescription interface
    public:
        bool validate() override;
        void update(double time, double period) override;
        void reset() override;

    };

    class VelocityLimitsImpl : public virtual VelocityLimits,
            public TaskDescriptionImpl
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(VelocityLimitsImpl)

        VelocityLimitsImpl(ModelInterface::ConstPtr model);
        VelocityLimitsImpl(YAML::Node yaml, ModelInterface::ConstPtr model);

        Eigen::VectorXd getQdotMax() const override;

    private:

        double _bound_scaling;
        Eigen::VectorXd _qdot_max;

        // TaskDescription interface
    public:
        bool validate() override;
        void update(double time, double period) override;
        void reset() override;

    };
    
} }


#endif

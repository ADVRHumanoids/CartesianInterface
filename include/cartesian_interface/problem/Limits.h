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

        bool setBoundScaling(double value);
        double getBoundScaling() const;

        Eigen::VectorXd getQmin() const;
        Eigen::VectorXd getQmax() const;

    private:

        double _bound_scaling;
        Eigen::VectorXd _qmin, _qmax;

        // TaskDescription interface
    public:
        bool validate() override;
        void update(double time, double period) override;
        void reset() override;

    };

    class VelocityLimits : public ConstraintDescription
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(VelocityLimits)

        VelocityLimits(ModelInterface::ConstPtr model);
        VelocityLimits(YAML::Node yaml, ModelInterface::ConstPtr model);

        Eigen::VectorXd getQdotMax() const;

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

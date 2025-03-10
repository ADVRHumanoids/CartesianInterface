#ifndef __XBOT_CARTESIAN_PROBLEM_OMNIWHEELS4X_H__
#define __XBOT_CARTESIAN_PROBLEM_OMNIWHEELS4X_H__

#include <cartesian_interface/sdk/problem/Constraint.h>
#include <cartesian_interface/Macro.h>

namespace XBot { namespace Cartesian {

    class OmniWheels4X : public virtual ConstraintDescription, public virtual TaskDescriptionImpl
    {

    public:

        CARTESIO_DECLARE_SMART_PTR(OmniWheels4X)

        OmniWheels4X(YAML::Node yaml, Context::ConstPtr context);

        double getl1() { return _l1; }
        double getl2() { return _l2; }
        double getr() { return _r; }
        const std::string& get_base_link() const { return _base_link; }
        const std::vector<std::string> get_joint_wheels_name() const { return _joint_wheels_name; }

    private:

        double _l1, _l2, _r;
        std::vector<std::string> _joint_wheels_name;
        std::string _base_link;
    };



} }


#endif

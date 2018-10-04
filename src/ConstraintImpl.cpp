#include <cartesian_interface/problem/Limits.h>

using namespace XBot::Cartesian;

ConstraintDescription::Ptr XBot::Cartesian::MakeJointLimits()
{
    return std::make_shared<ConstraintDescription>("JointLimits");
}

ConstraintDescription::Ptr XBot::Cartesian::MakeVelocityLimits()
{
    return std::make_shared<ConstraintDescription>("VelocityLimits");
}
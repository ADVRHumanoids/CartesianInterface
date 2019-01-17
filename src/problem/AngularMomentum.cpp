#include <cartesian_interface/problem/AngularMomentum.h>

using namespace XBot::Cartesian;


AngularMomentumTask::Ptr XBot::Cartesian::MakeAngularMomentum()
{
    return std::make_shared<AngularMomentumTask>();
}

AngularMomentumTask::Ptr XBot::Cartesian::GetAsAngularMomentum(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<AngularMomentumTask>(task);
}

XBot::Cartesian::AngularMomentumTask::AngularMomentumTask():
    TaskDescription(TaskInterface::None, "AngularMomentum", 3),
    min_rate(false)
{

}

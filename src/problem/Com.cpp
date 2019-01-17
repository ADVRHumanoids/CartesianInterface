#include <cartesian_interface/problem/Com.h>

using namespace XBot::Cartesian;


ComTask::ComTask():
    CartesianTask("com", "world", 3, "Com")
{

}

ComTask::Ptr XBot::Cartesian::MakeCom()
{
    return std::make_shared<ComTask>();
}

ComTask::Ptr XBot::Cartesian::GetAsCom(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<ComTask>(task);
}

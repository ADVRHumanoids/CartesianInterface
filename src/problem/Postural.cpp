#include <cartesian_interface/problem/Postural.h>

using namespace XBot::Cartesian;

PosturalTask::Ptr XBot::Cartesian::GetAsPostural(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<PosturalTask>(task);
}

PosturalTask::PosturalTask(int ndof):
    TaskDescription(TaskInterface::Postural, "Postural", ndof),
    use_inertia_matrix(false)
{

}

PosturalTask::Ptr XBot::Cartesian::MakePostural(int ndof)
{
    return std::make_shared<PosturalTask>(ndof);
}

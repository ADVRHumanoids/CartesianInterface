#include <cartesian_interface/problem/Interaction.h>

using namespace XBot::Cartesian;


InteractionTask::Ptr XBot::Cartesian::GetAsInteraction(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<InteractionTask>(task);
}

InteractionTask::InteractionTask(std::string distal_link,
                                 std::string base_link,
                                 int size,
                                 std::string type):
    CartesianTask(distal_link, base_link, 6, type)
{
    interface = TaskInterface::Interaction;
}

InteractionTask::Ptr XBot::Cartesian::MakeInteraction(std::string distal_link,
                                                      std::string base_link)
{
    return std::make_shared<InteractionTask>(distal_link, base_link, 6, "Interaction");
}

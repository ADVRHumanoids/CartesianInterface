#include <cartesian_interface/problem/Gaze.h>

using namespace XBot::Cartesian;


GazeTask::GazeTask(std::string base_link):
    CartesianTask("gaze", base_link, 2, "Gaze") //HERE THE SIZE IS 2 BECAUSE THE GAZE IS CONSIDERED IMPLEMENTED USING PAN-TILT
{

}

GazeTask::Ptr XBot::Cartesian::MakeGaze(std::string base_link)
{
    return std::make_shared<GazeTask>(base_link);
}

GazeTask::Ptr XBot::Cartesian::GetAsGaze(TaskDescription::Ptr task)
{
    return std::dynamic_pointer_cast<GazeTask>(task);
}

TaskDescription::Ptr GazeTask::yaml_parse_gaze(YAML::Node task_node, ModelInterface::ConstPtr model)
{

    std::string base_link = "world";

    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }

    return MakeGaze(base_link);
}

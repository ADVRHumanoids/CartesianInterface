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

TaskDescription::Ptr InteractionTask::yaml_parse_interaction(YAML::Node task_node, ModelInterface::ConstPtr model)
{
    std::string distal_link = task_node["distal_link"].as<std::string>();
    std::string base_link = "world";


    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }

    std::vector<double> stiffness, damping;

    if(task_node["stiffness"])
    {
        stiffness = task_node["stiffness"].as<std::vector<double>>();
        if(stiffness.size() != 6)
        {
            throw std::runtime_error("'stiffness' field for interaction tasks requires six values");
        }
    }
    else
    {
        throw std::runtime_error("'stiffness' field required for interaction tasks (6 values)");
    }


    if(task_node["damping"])
    {
        damping = task_node["damping"].as<std::vector<double>>();
        if(damping.size() != 6)
        {
            throw std::runtime_error("'damping' field for interaction tasks requires six values");
        }
    }
    else
    {
        throw std::runtime_error("'damping' field required for interaction tasks (6 values)");
    }


    auto i_task = MakeInteraction(distal_link, base_link);

    i_task->stiffness = Eigen::Vector6d::Map(stiffness.data());
    i_task->damping = Eigen::Vector6d::Map(damping.data());

    if(task_node["force_estimation_chains"])
    {
        i_task->force_estimation_chains = task_node["force_estimation_chains"].as<std::vector<std::string>>();
    }

    TaskDescription::Ptr task_desc = i_task;

    if(task_node["orientation_gain"])
    {
        i_task->orientation_gain = task_node["orientation_gain"].as<double>();
    }

    return task_desc;
}

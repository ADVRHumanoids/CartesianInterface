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
    force_dead_zone.setZero();
    inertia.setZero();
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
    
    auto i_task = MakeInteraction(distal_link, base_link);

    std::vector<double> stiffness, damping, inertia, deadzone;

    if(task_node["stiffness"])
    {
        stiffness = task_node["stiffness"].as<std::vector<double>>();
        if(stiffness.size() != 6)
        {
            throw std::runtime_error("'stiffness' field for interaction tasks requires six values");
        }
        i_task->stiffness = Eigen::Vector6d::Map(stiffness.data());
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
        i_task->damping = Eigen::Vector6d::Map(damping.data());
    }
    else
    {
        throw std::runtime_error("'damping' field required for interaction tasks (6 values)");
    }
    
    if(task_node["force_dead_zone"])
    {
        deadzone = task_node["force_dead_zone"].as<std::vector<double>>();
        if(deadzone.size() != 6)
        {
            throw std::runtime_error("'force_dead_zone' field for interaction tasks requires six values");
        }
        i_task->force_dead_zone = Eigen::Vector6d::Map(deadzone.data());
    }
    
    if(task_node["inertia"])
    {
        inertia = task_node["inertia"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'inertia' field for interaction tasks requires six values");
        }
        i_task->inertia = Eigen::Vector6d::Map(inertia.data());
    }


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

#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/utils/TaskFactory.h>

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

TaskDescription::Ptr PosturalTask::yaml_parse_postural(YAML::Node task_node,
                                                   ModelInterface::ConstPtr model)
{
    auto task_desc = MakePostural(model->getJointNum());
    auto postural_desc = GetAsPostural(task_desc);

    if(task_node["weight"] && task_node["weight"].IsMap())
    {
        for(const auto& pair : task_node["weight"])
        {
            if(!model->hasJoint(pair.first.as<std::string>()))
            {
                std::string err = "Joint " + pair.first.as<std::string>() + " is undefined";
                throw std::invalid_argument(err);
            }

            int idx = model->getDofIndex(pair.first.as<std::string>());
            task_desc->weight(idx, idx) = pair.second.as<double>();

        }
    }

    if(task_node["use_inertia"] && task_node["use_inertia"].as<bool>())
    {
        postural_desc->use_inertia_matrix = true;
    }

    return task_desc;
}


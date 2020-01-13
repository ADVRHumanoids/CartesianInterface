#include <cartesian_interface/problem/Postural.h>
#include <cartesian_interface/utils/TaskFactory.h>

using namespace XBot::Cartesian;

PosturalTask::PosturalTask(ModelInterface::ConstPtr model, int ndof):
    TaskDescription("Postural", "postural", ndof, model),
    use_inertia_matrix(false)
{

}

PosturalTask::PosturalTask(YAML::Node task_node,
                           ModelInterface::ConstPtr model):
    TaskDescription(task_node, model, "postural", model->getJointNum())
{
    Eigen::MatrixXd weight;
    weight.setIdentity(getSize(), getSize());

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
            weight(idx, idx) = pair.second.as<double>();

        }

        setWeight(weight);
    }
    
    if(task_node["indices"])
    {
        throw std::runtime_error("Specifying indices is not allowed for postural tasks. Use disabled_joints instead.");
    }

    if(task_node["use_inertia"] && task_node["use_inertia"].as<bool>())
    {
        use_inertia_matrix = true;
    }

}

void PosturalTask::run(double time, double period)
{

}

void PosturalTask::reset()
{
    _model->getJointPosition(_qref);
}


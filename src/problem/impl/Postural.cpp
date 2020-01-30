#include <cartesian_interface/utils/TaskFactory.h>

#include "Postural.h"

using namespace XBot::Cartesian;

PosturalTaskImpl::PosturalTaskImpl(ModelInterface::ConstPtr model, int ndof):
    TaskDescriptionImpl("Postural", "Postural", ndof, model),
    _use_inertia_matrix(false)
{

}

PosturalTaskImpl::PosturalTaskImpl(YAML::Node task_node,
                           ModelInterface::ConstPtr model):
    TaskDescriptionImpl(task_node, model, "Postural", model->getJointNum()),
    _use_inertia_matrix(false)
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
        _use_inertia_matrix = true;
    }

}

bool PosturalTaskImpl::useInertiaMatrixWeight() const
{
    return _use_inertia_matrix;
}

void PosturalTaskImpl::getReferencePosture(Eigen::VectorXd & qref) const
{
    qref = _qref;
}

void PosturalTaskImpl::getReferencePosture(XBot::JointNameMap & qref) const
{
    _model->eigenToMap(_qref, qref);
}

void PosturalTaskImpl::setReferencePosture(const XBot::JointNameMap & qref)
{
    _model->mapToEigen(qref, _qref);
}

void PosturalTaskImpl::update(double time, double period)
{
    TaskDescriptionImpl::update(time, period);
}

void PosturalTaskImpl::reset()
{
    _model->getJointPosition(_qref);
}



void PosturalTaskImpl::setIndices(const std::vector<int>& value)
{
    TaskDescriptionImpl::setIndices(value);

    // TBD act on disabled joints
}

void PosturalTaskImpl::setDisabledJoints(const std::vector<std::string>& value)
{
    TaskDescriptionImpl::setDisabledJoints(value);

    // TBD act on indices
}

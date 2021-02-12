#include <cartesian_interface/utils/TaskFactory.h>

#include "problem/Postural.h"

using namespace XBot::Cartesian;

PosturalTaskImpl::PosturalTaskImpl(Context::ConstPtr context, int ndof):
    TaskDescriptionImpl("Postural", "Postural", ndof, context),
    _use_inertia_matrix(false)
{
    reset();
}

PosturalTaskImpl::PosturalTaskImpl(YAML::Node task_node,
                                   Context::ConstPtr context):
    TaskDescriptionImpl(task_node,
                        context,
                        "Postural",
                        context->model()->getJointNum()),
    _use_inertia_matrix(false)
{
    Eigen::MatrixXd weight;
    weight.setIdentity(getSize(), getSize());

    if(task_node["weight"] && task_node["weight"].IsMap())
    {
        for(const auto& pair : task_node["weight"])
        {
            if(!_model->hasJoint(pair.first.as<std::string>()))
            {
                std::string err = "Joint " + pair.first.as<std::string>() + " is undefined";
                throw std::invalid_argument(err);
            }

            int idx = _model->getDofIndex(pair.first.as<std::string>());
            weight(idx, idx) = pair.second.as<double>();

        }

        setWeight(weight);
    }
    
    if(task_node["indices"])
    {
        throw std::runtime_error("specifying indices is not allowed for postural tasks: "
                                 "use disabled_joints instead");
    }

    if(task_node["use_inertia"] && task_node["use_inertia"].as<bool>())
    {
        _use_inertia_matrix = true;
    }

    setDisabledJoints(getDisabledJoints());

    reset();
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

void PosturalTaskImpl::getReferenceVelocity(Eigen::VectorXd& qdotref) const
{
    qdotref = _qdotref;
}

void PosturalTaskImpl::setReferencePosture(const XBot::JointNameMap & qref)
{
    _model->mapToEigen(qref, _qref);
}

void PosturalTaskImpl::setReferenceVelocity(const XBot::JointNameMap& qdotref)
{
    _model->mapToEigen(qdotref, _qdotref);
}

void PosturalTaskImpl::setReferenceVelocity(const Eigen::VectorXd& qdotref)
{
    if(qdotref.size() == _qdotref.size())
    {
        _qdotref = qdotref;
        return;
    }

    Logger::error("[PosturalTaskImpl] qdotref.size() = %d != _qdotref.size() = %d \n",
                  qdotref.size(), _qdotref.size());
}

void PosturalTaskImpl::update(double time, double period)
{
    TaskDescriptionImpl::update(time, period);
}

void PosturalTaskImpl::reset()
{
    _model->getJointPosition(_qref);
    _qdotref.setZero(_qref.size());
}



void PosturalTaskImpl::setIndices(const std::vector<int>&)
{
    throw std::runtime_error("specifying indices is not allowed for postural tasks: "
                             "use disabled_joints instead");
}

void PosturalTaskImpl::setDisabledJoints(const std::vector<std::string>& value)
{
    TaskDescriptionImpl::setDisabledJoints(value);

    std::vector<int> indices;
    for(int i = 0; i < getSize(); i++)
    {
        indices.push_back(i);
    }

    auto to_be_removed = [this, value](auto i)
    {
        return std::find(value.begin(), value.end(),
                  _model->getEnabledJointNames().at(i)) != value.end();
    };

    auto first_invalid = std::remove_if(indices.begin(),
                                        indices.end(),
                                        to_be_removed);

    indices.erase(first_invalid, indices.end());

    TaskDescriptionImpl::setIndices(indices);
}

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
        throw std::runtime_error("Specifying indices is not allowed for postural tasks. Use disabled_joints instead.");
    }

    if(task_node["use_inertia"] && task_node["use_inertia"].as<bool>())
    {
        _use_inertia_matrix = true;
    }

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
    if(qdotref.size() == _qdotref.size())
    {
        qdotref = _qdotref;
    }
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
        _qdotref = qdotref;
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

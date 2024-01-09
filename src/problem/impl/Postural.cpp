#include <cartesian_interface/utils/TaskFactory.h>
#include <xbot2_interface/logger.h>
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
                        context->model()->getNv()),
    _use_inertia_matrix(false)
{
    Eigen::MatrixXd weight;
    weight.setIdentity(getSize(), getSize());

    if(task_node["weight"] && task_node["weight"].IsMap())
    {
        for(const auto& pair : task_node["weight"])
        {
            int idx = _model->getVIndexFromVName(pair.first.as<std::string>());

            if(idx < 0)
            {
                std::string err = "Joint " + pair.first.as<std::string>() + " is undefined";
                throw std::invalid_argument(err);
            }

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
    _model->qToMap(_qref, qref);
}

void PosturalTaskImpl::getReferenceVelocity(Eigen::VectorXd& qdotref) const
{
    qdotref = _qdotref;
}

void PosturalTaskImpl::setReferencePosture(const XBot::JointNameMap & qref)
{
    _model->mapToQ(qref, _qref);
}

void PosturalTaskImpl::setReferencePosture(const Eigen::VectorXd& qref)
{
    if(qref.size() == _qref.size())
    {
        _qref = qref;
        return;
    }
}

void PosturalTaskImpl::setReferenceVelocity(const XBot::JointNameMap& qdotref)
{
    _model->mapToV(qdotref, _qdotref);
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
    _qdotref.setZero(_model->getNv());
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
        bool vname_found = value.end() !=
                           std::find(value.begin(), value.end(), _model->getVNames().at(i));

        bool jname_found = value.end() !=
                           std::find_if(value.begin(), value.end(), [i, this](const auto &item) {
            auto jinfo = _model->getJointInfo(item);
            return i >= jinfo.iv && i < jinfo.iv + jinfo.nv;
        });

        return jname_found || vname_found;
    };

    auto first_invalid = std::remove_if(indices.begin(),
                                        indices.end(),
                                        to_be_removed);

    indices.erase(first_invalid, indices.end());

    TaskDescriptionImpl::setIndices(indices);
}

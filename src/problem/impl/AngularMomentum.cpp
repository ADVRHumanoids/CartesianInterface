#include <cartesian_interface/utils/TaskFactory.h>

#include "problem/AngularMomentum.h"

using namespace XBot::Cartesian;

AngularMometumTaskImpl::AngularMometumTaskImpl(Context::ConstPtr context):
    TaskDescriptionImpl("AngularMomentum", "AngularMomentum", 3, context)
{
    reset();
}

AngularMometumTaskImpl::AngularMometumTaskImpl(YAML::Node task_node, Context::ConstPtr context):
    TaskDescriptionImpl(task_node, context, "AngularMomentum", 3)
{
    reset();
}

void AngularMometumTaskImpl::update(double time, double period)
{
    TaskDescriptionImpl::update(time, period);
}

void AngularMometumTaskImpl::getAngularMomentumReference(Eigen::Vector3d &href) const
{
    href = _href;
}

void AngularMometumTaskImpl::setAngularMomentumReference(const Eigen::Vector3d &href)
{
    _href = href;
}

void AngularMometumTaskImpl::reset()
{
    _model->getCentroidalMomentum(_momentum);
    _href = _momentum.tail(3);
}

void AngularMometumTaskImpl::setIndices(const std::vector<int>& value) ///is it needed?
{
    TaskDescriptionImpl::setIndices(value);

    // TBD act on disabled joints
}

void AngularMometumTaskImpl::setDisabledJoints(const std::vector<std::string>& value) ///is it needed?
{
    TaskDescriptionImpl::setDisabledJoints(value);

    // TBD act on indices
}

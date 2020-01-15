#include "OpenSotPostural.h"

using namespace XBot::Cartesian;


OpenSotPosturalAdapter::OpenSotPosturalAdapter(TaskDescription::Ptr task,
                                               XBot::ModelInterface::ConstPtr model):
    OpenSotTaskAdapter(task, model)
{
    _ci_postural = std::dynamic_pointer_cast<PosturalTask>(task);

    if(!_ci_postural) throw std::runtime_error("Provided task description "
                                               "does not have expected type 'PosturalTask'");
}

TaskPtr OpenSotPosturalAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    return _opensot_postural = boost::make_shared<PosturalSoT>(q);
}

bool OpenSotPosturalAdapter::initialize()
{

    _use_inertia_matrix = _ci_postural->use_inertia_matrix;

    return true;
}

void OpenSotPosturalAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    if(_use_inertia_matrix)
    {
        _model->getInertiaMatrix(_inertia_matrix);
        _opensot_postural->setWeight(_inertia_matrix);
    }

    _ci_postural->getReferencePosture(_qref);
    _opensot_postural->setReference(_qref);
}

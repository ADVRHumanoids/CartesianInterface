#include "opensot/OpenSotPostural.h"

using namespace XBot::Cartesian;


OpenSotPosturalAdapter::OpenSotPosturalAdapter(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context),
    _use_inertia_matrix(false)
{
    _ci_postural = std::dynamic_pointer_cast<PosturalTask>(task);

    if(!_ci_postural) throw std::runtime_error("Provided task description "
                                               "does not have expected type 'PosturalTask'");
}

TaskPtr OpenSotPosturalAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_postural = SotUtils::make_shared<PosturalSoT>(*_model, q);

    return _opensot_postural;
}

bool OpenSotPosturalAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotPosturalAdapter] requires default variables definition");
    }

    if(!OpenSotTaskAdapter::initialize(vars))
    {
        return false;
    }

    _use_inertia_matrix = _ci_postural->useInertiaMatrixWeight();

    return true;
}

void OpenSotPosturalAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    if(_use_inertia_matrix)
    {
        _model->computeInertiaMatrix(_inertia_matrix);
        _opensot_postural->setWeight(_inertia_matrix);
    }

    _ci_postural->getReferencePosture(_qref);
    _ci_postural->getReferenceVelocity(_qdotref);
    _qdotref *= _ctx->params()->getControlPeriod();
    _opensot_postural->setReference(_qref, _qdotref);
}

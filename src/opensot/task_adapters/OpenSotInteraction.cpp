#include "opensot/OpenSotInteraction.h"

using namespace XBot::Cartesian;

estimation_utils::ForceEstimation::WeakPtr OpenSotInteractionAdapter::_fest_weak;

OpenSotInteractionAdapter::OpenSotInteractionAdapter(TaskDescription::Ptr task,
                                                     Context::ConstPtr context):
    OpenSotCartesianAdapter(task, context),
    _fest_upd(false)
{
    _ci_adm = std::dynamic_pointer_cast<AdmittanceTask>(task);

    if(!_ci_adm) throw std::runtime_error("Provided task description "
                                          "does not have expected type 'AdmittanceTask'");


}

TaskPtr OpenSotInteractionAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    XBot::ForceTorqueSensor::ConstPtr ft;

    try
    {
        ft = _model->getForceTorque().at(_ci_adm->getDistalLink());
    }
    catch(std::out_of_range& e)
    {
        /* We create a force estimator just once */
        _fest = _fest_weak.lock();
        if(!_fest)
        {
            _fest = std::make_shared<estimation_utils::ForceEstimation>(_model);
            _fest_weak = _fest;
            _fest_upd = true;
        }

        ft = _fest->add_link(_ci_adm->getDistalLink(),
        {},
                             _ci_adm->getForceEstimationChains());
    }


    _opensot_adm = SotUtils::make_shared<AdmittanceSoT>
            ("adm_" + _ci_adm->getName(),
             const_cast<ModelInterface&>(*_model),
             _ci_adm->getBaseLink(),
             ft
             );

    _opensot_adm->setDeadZone(_ci_adm->getForceDeadzone());

    _opensot_cart = _opensot_adm;

    return _opensot_adm;
}

bool OpenSotInteractionAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    return OpenSotCartesianAdapter::initialize(vars);
}

void OpenSotInteractionAdapter::update(double time, double period)
{
    OpenSotCartesianAdapter::update(time, period);

    if(_fest_upd)
    {
        _fest->update();
    }

}

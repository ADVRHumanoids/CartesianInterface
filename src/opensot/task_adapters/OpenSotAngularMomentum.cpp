#include "opensot/OpenSotAngularMomentum.h"

using namespace XBot::Cartesian;

OpenSotAngularMomentumAdapter::OpenSotAngularMomentumAdapter(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_angular_mom = std::dynamic_pointer_cast<AngularMomentumTask>(task);

    if(!_ci_angular_mom) throw std::runtime_error("Provided task description "
                                               "does not have expected type 'AngularMomentumTask'");
}

TaskPtr OpenSotAngularMomentumAdapter::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _opensot_angular_mom = boost::make_shared<AngularMomSoT>(q, const_cast<ModelInterface&>(*_model));

    return _opensot_angular_mom;
}

bool OpenSotAngularMomentumAdapter::initialize(const OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotAngularMomentumAdapter] requires default variables definition");
    }

    if(!OpenSotTaskAdapter::initialize(vars))
    {
        return false;
    }

    return true;
}

void OpenSotAngularMomentumAdapter::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);


    _ci_angular_mom->getAngularMomentumReference(_href);
    _opensot_angular_mom->setReference(_href);
}


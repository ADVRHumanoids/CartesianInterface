#include "OpenSotAngularMomentum.h"

#include <boost/make_shared.hpp>

#include "fmt/format.h"

using namespace XBot::Cartesian;

OpenSotAngularMomentum::OpenSotAngularMomentum(TaskDescription::Ptr task,
                                               Context::ConstPtr context):
    OpenSotTaskAdapter(task, context)
{
    _ci_angmom = std::dynamic_pointer_cast<AngularMomentum>(task);

    if(!_ci_angmom) throw std::runtime_error("Provided task description "
                                             "does not have expected type 'AngularMomentum'");

}

TaskPtr OpenSotAngularMomentum::constructTask()
{
    Eigen::VectorXd q;
    _model->getJointPosition(q);

    _sot_angmom = SotUtils::make_shared<AngularMomentumSoT>(q,
                                                         const_cast<ModelInterface&>(*_model));

    return _sot_angmom;
}

bool OpenSotAngularMomentum::initialize(const ::OpenSoT::OptvarHelper& vars)
{
    if(vars.getAllVariables().size() > 0)
    {
        throw BadVariables("[OpenSotAngularMomentum] requires default variables definition");
    }

    return OpenSotTaskAdapter::initialize(vars);
}

void OpenSotAngularMomentum::update(double time, double period)
{
    OpenSotTaskAdapter::update(time, period);

    _sot_angmom->setReference(_ci_angmom->getReference() * period);
}

CARTESIO_REGISTER_OPENSOT_TASK_PLUGIN(OpenSotAngularMomentum, AngularMomentum)


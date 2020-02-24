#ifndef OPENSOTINTERACTION_H
#define OPENSOTINTERACTION_H

#include "OpenSotCartesian.h"
#include <cartesian_interface/problem/Interaction.h>

#include <boost/make_shared.hpp>

#include <OpenSoT/tasks/velocity/CartesianAdmittance.h>

#include <cartesian_interface/utils/estimation/ForceEstimation.h>

using AdmittanceSoT = OpenSoT::tasks::velocity::CartesianAdmittance;

namespace XBot { namespace Cartesian {

class OpenSotInteractionAdapter :
        public OpenSotCartesianAdapter
{

public:

    OpenSotInteractionAdapter(TaskDescription::Ptr task,
                              ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

protected:

    AdmittanceSoT::Ptr _opensot_adm;

private:

    AdmittanceTask::Ptr _ci_adm;
    bool _fest_upd;
    Utils::ForceEstimation::Ptr _fest;

    static Utils::ForceEstimation::WeakPtr _fest_weak;

};

} }

#endif // OPENSOTINTERACTION_H

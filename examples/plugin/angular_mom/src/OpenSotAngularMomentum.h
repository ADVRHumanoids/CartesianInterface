#ifndef OPENSOTANGULARMOMENTUM_H
#define OPENSOTANGULARMOMENTUM_H

#include "AngularMomentum.h"
#include <cartesian_interface/sdk/opensot/OpenSotTask.h>

#include <OpenSoT/tasks/velocity/AngularMomentum.h>

using AngularMomentumSoT = OpenSoT::tasks::velocity::AngularMomentum;

namespace XBot { namespace Cartesian {


class OpenSotAngularMomentum : public OpenSotTaskAdapter
{

public:

    OpenSotAngularMomentum(TaskDescription::Ptr task,
                           ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize() override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotAngularMomentum() override = default;

private:

    AngularMomentumSoT::Ptr _sot_angmom;
    AngularMomentum::Ptr _ci_angmom;


};

} }

#endif // OPENSOTANGULARMOMENTUM_H

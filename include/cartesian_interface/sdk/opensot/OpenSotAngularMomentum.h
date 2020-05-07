#ifndef OPENSOTANGULARMOMENTUM_H
#define OPENSOTANGULARMOMENTUM_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/AngularMomentum.h>
#include "OpenSotTask.h"

#include <OpenSoT/tasks/velocity/AngularMomentum.h>

using AngularMomSoT = OpenSoT::tasks::velocity::AngularMomentum;

namespace XBot { namespace Cartesian {

class OpenSotAngularMomentumAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotAngularMomentumAdapter(TaskDescription::Ptr task,
                      Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotAngularMomentumAdapter() override = default;

protected:

private:

    AngularMomSoT::Ptr _opensot_angular_mom;
    AngularMomentumTask::Ptr _ci_angular_mom;

    Eigen::Vector3d _href;
};

} }
#endif // OPENSOTPOSTURAL_H

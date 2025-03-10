#ifndef OPENSOTGAZEADAPTER_H
#define OPENSOTGAZEADAPTER_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Gaze.h>
#include "OpenSotTask.h"

#include <OpenSoT/tasks/velocity/Gaze.h>

using GazeSoT = OpenSoT::tasks::velocity::Gaze;

namespace XBot { namespace Cartesian {

class OpenSotGazeAdapter :
        public OpenSotTaskAdapter,
        public virtual CartesianTaskObserver
{

public:

    OpenSotGazeAdapter(TaskDescription::Ptr task,
                            Context::ConstPtr context);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    bool onBaseLinkChanged() override;

    bool onControlModeChanged() override;

    virtual ~OpenSotGazeAdapter() override = default;

protected:

    GazeSoT::Ptr _opensot_cart;

private:

    GazeTask::Ptr _ci_cart;
};




} }

#endif // OPENSOTTASKADAPTER_H

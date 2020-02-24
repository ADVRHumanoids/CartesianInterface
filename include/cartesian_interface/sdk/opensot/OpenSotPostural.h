#ifndef OPENSOTPOSTURAL_H
#define OPENSOTPOSTURAL_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Postural.h>
#include "OpenSotTask.h"

#include <OpenSoT/tasks/velocity/Postural.h>

using PosturalSoT = OpenSoT::tasks::velocity::Postural;

namespace XBot { namespace Cartesian {

class OpenSotPosturalAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotPosturalAdapter(TaskDescription::Ptr task,
                      ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize(const OpenSoT::OptvarHelper& vars) override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotPosturalAdapter() override = default;

protected:

private:

    PosturalSoT::Ptr _opensot_postural;
    PosturalTask::Ptr _ci_postural;

    bool _use_inertia_matrix;
    Eigen::MatrixXd _inertia_matrix;
    Eigen::VectorXd _qref;
};

} }
#endif // OPENSOTPOSTURAL_H

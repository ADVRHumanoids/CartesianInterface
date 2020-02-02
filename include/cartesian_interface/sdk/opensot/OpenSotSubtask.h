#ifndef OPENSOTSUBTASK_H
#define OPENSOTSUBTASK_H

#include <boost/make_shared.hpp>

#include <cartesian_interface/problem/Subtask.h>
#include "OpenSotTask.h"

#include <OpenSoT/SubTask.h>

using SubtaskSoT = OpenSoT::SubTask;

namespace XBot { namespace Cartesian {

class OpenSotSubtaskAdapter :
        public OpenSotTaskAdapter
{

public:

    OpenSotSubtaskAdapter(TaskDescription::Ptr task,
                          ModelInterface::ConstPtr model);

    virtual TaskPtr constructTask() override;

    virtual bool initialize() override;

    virtual void update(double time, double period) override;

    virtual ~OpenSotSubtaskAdapter() override = default;

private:

    SubtaskSoT::Ptr _sot_subtask;
    Subtask::Ptr _ci_subtask;
    OpenSotTaskAdapter::Ptr _task_adapter;

};

} }

#endif // OPENSOTSUBTASK_H

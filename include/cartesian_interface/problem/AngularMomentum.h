#ifndef __XBOT_PROBLEM_ANGULAR_MOMENTUM_H__
#define __XBOT_PROBLEM_ANGULAR_MOMENTUM_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {

    struct AngularMomentumTask: TaskDescription{
        typedef std::shared_ptr<AngularMomentumTask> Ptr;
        typedef std::shared_ptr<const AngularMomentumTask> ConstPtr;

        AngularMomentumTask();
    };

    AngularMomentumTask::Ptr MakeAngularMomentum();

    AngularMomentumTask::Ptr GetAsAngularMomentum(TaskDescription::Ptr task);

}
}

#endif

#ifndef __XBOT_PROBLEM_ANGULAR_MOMENTUM_H__
#define __XBOT_PROBLEM_ANGULAR_MOMENTUM_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {

    struct AngularMomentumTask: TaskDescription{
        typedef std::shared_ptr<AngularMomentumTask> Ptr;
        typedef std::shared_ptr<const AngularMomentumTask> ConstPtr;

        /**
         * @brief min_rate minimize the rate of change of the angular momentum.
         * To use it set the tag:
         *                          min_rate: true
         */
        bool min_rate;

        AngularMomentumTask();
    };

    AngularMomentumTask::Ptr MakeAngularMomentum();

    AngularMomentumTask::Ptr GetAsAngularMomentum(TaskDescription::Ptr task);

}
}

#endif

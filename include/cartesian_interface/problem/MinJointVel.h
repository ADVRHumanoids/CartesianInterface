#ifndef __XBOT_CARTESIAN_PROBLEM_MINJOINTVEL_H__
#define __XBOT_CARTESIAN_PROBLEM_MINJOINTVEL_H__

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {

struct MinJointVelTask : TaskDescription {
    typedef std::shared_ptr<MinJointVelTask> Ptr;
    typedef std::shared_ptr<const MinJointVelTask> ConstPtr;

    MinJointVelTask(int ndof);

    double eps;

    static TaskDescription::Ptr yaml_parse_minjointvel(YAML::Node node, ModelInterface::ConstPtr model);
};

MinJointVelTask::Ptr MakeMinJointVel(int ndof);

MinJointVelTask::Ptr GetAsMinJointVel(TaskDescription::Ptr task);

}
}

#endif

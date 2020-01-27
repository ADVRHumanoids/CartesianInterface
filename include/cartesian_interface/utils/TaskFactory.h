#ifndef __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__
#define __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>

#include <cstdio>
#include <dlfcn.h>

namespace XBot { namespace Cartesian { 

class TaskFactory
{

public:

    TaskFactory(YAML::Node prob_desc,
                ModelInterface::ConstPtr model);

    TaskDescription::Ptr makeTask(std::string task_name);

private:

    YAML::Node _prob_desc;
    ModelInterface::ConstPtr _model;
    std::map<std::string, TaskDescription::Ptr> _subtask_map;
};

std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node prob_desc,
                                                     std::string task_name,
                                                     ModelInterface::ConstPtr model
                                                     );

std::shared_ptr<ConstraintDescription> MakeConstraintDescription(YAML::Node constr_node,
                                                                 ModelInterface::ConstPtr model,
                                                                 std::string lib_name
                                                                 );

struct BadTaskDescription : public std::runtime_error
{
    BadTaskDescription(std::string what);
};

struct TaskIsSubtask : public std::exception
{
    std::string real_task_name;
    std::vector<int> indices;
};

} } 

#endif

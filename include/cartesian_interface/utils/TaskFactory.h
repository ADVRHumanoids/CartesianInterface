#ifndef __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__
#define __XBOT_CARTESIAN_UTILS_TASK_FACTORY_H__

#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/problem/Constraint.h>
#include <cartesian_interface/Context.h>

#include <cstdio>
#include <dlfcn.h>

namespace XBot { namespace Cartesian { 

class TaskFactory
{

public:

    TaskFactory(YAML::Node prob_desc,
                Context::ConstPtr context);

    TaskDescription::Ptr makeTask(std::string task_name);

private:

    YAML::Node _prob_desc;
    Context::ConstPtr _context;
    std::map<std::string, TaskDescription::Ptr> _subtask_map;
};

std::shared_ptr<TaskDescription> MakeTaskDescription(YAML::Node prob_desc,
                                                     std::string task_name,
                                                     Context::ConstPtr context
                                                     );

std::shared_ptr<ConstraintDescription> MakeConstraintDescription(YAML::Node constr_node,
                                                                 ModelInterface::ConstPtr model,
                                                                 Context::ConstPtr context
                                                                 );

struct BadTaskDescription : public std::runtime_error
{
    BadTaskDescription(std::string what);
};

struct TaskIsSubtask : public std::exception
{
    std::string real_task_name;
    std::vector<int> indices;
    std::vector<int> remove_indices;
};

} } 

#endif

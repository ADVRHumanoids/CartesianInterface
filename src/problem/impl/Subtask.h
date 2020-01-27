#ifndef SUBTASK_IMPL_H
#define SUBTASK_IMPL_H

#include <cartesian_interface/problem/Subtask.h>
#include "Task.h"

namespace XBot { namespace Cartesian {


class SubtaskImpl : public virtual Subtask,
        public TaskDescriptionImpl
{

public:


    SubtaskImpl(TaskDescriptionImpl::Ptr task,
                std::vector<int> indices,
                std::string subtask_name = "");


    TaskDescription::Ptr getTask() override;


private:

    static std::string gen_subtask_name(TaskDescriptionImpl::Ptr task,
                                    std::vector<int> indices);

    TaskDescriptionImpl::Ptr _task;

};

} }

#endif // SUBTASK_H

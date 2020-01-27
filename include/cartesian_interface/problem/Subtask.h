#ifndef SUBTASK_H
#define SUBTASK_H

#include <cartesian_interface/problem/Task.h>

namespace XBot { namespace Cartesian {

class Subtask : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(Subtask)

    virtual TaskDescription::Ptr getTask() = 0;

    virtual const std::vector<int>& getSubtaskIndices() const = 0;

};

} }

#endif // SUBTASK_H

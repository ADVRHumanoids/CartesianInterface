#ifndef SUBTASK_IMPL_H
#define SUBTASK_IMPL_H

#include <cartesian_interface/problem/Subtask.h>
#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {


class SubtaskImpl : public virtual Subtask,
        public TaskDescriptionImpl
{

public:


    SubtaskImpl(TaskDescriptionImpl::Ptr task,
                std::vector<int> indices,
                std::string subtask_name = "");


    TaskDescription::Ptr getTask() override;

    const Eigen::MatrixXd& getWeight() const override;
    bool setWeight(const Eigen::MatrixXd& value) override;

    const std::vector<int>& getSubtaskIndices() const override;

    void reset() override;

private:

    static std::string gen_subtask_name(TaskDescriptionImpl::Ptr task,
                                    std::vector<int> indices);

    TaskDescriptionImpl::Ptr _task;

    std::vector<int> _indices;

    mutable Eigen::MatrixXd _tmp_weight_small, _tmp_weight_big;
};

} }

#endif // SUBTASK_H

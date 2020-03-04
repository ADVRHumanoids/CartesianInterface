#include "problem/Subtask.h"

using namespace XBot::Cartesian;

namespace
{

void weightSmallToBig(const Eigen::MatrixXd& Wsmall,
                      Eigen::MatrixXd& Wbig,
                      const std::vector<int>& indices)
{
    int i = 0;
    for(auto idx_i : indices)
    {
        int j = 0;
        for(auto idx_j : indices)
        {
            Wbig(idx_i, idx_j) = Wsmall(i, j);
            j++;
        }
        i++;
    }
}

void weightBigToSmall(const Eigen::MatrixXd& Wbig,
                      Eigen::MatrixXd& Wsmall,
                      const std::vector<int>& indices)
{
    int i = 0;
    for(auto idx_i : indices)
    {
        int j = 0;
        for(auto idx_j : indices)
        {
            Wsmall(i, j) = Wbig(idx_i, idx_j);
            j++;
        }
        i++;
    }
}

}


SubtaskImpl::SubtaskImpl(TaskDescriptionImpl::Ptr task,
                         std::vector<int> indices,
                         std::string subtask_name):
    TaskDescriptionImpl("Subtask",
                        subtask_name == "" ? gen_subtask_name(task, indices) : subtask_name,
                        indices.size(),
                        task->getModel()),
    _task(task),
    _indices(indices),
    _tmp_weight_small(indices.size(), indices.size()),
    _tmp_weight_big(task->getSize(), task->getSize())
{

}

std::string SubtaskImpl::gen_subtask_name(TaskDescriptionImpl::Ptr task, std::vector<int> indices)
{
    std::stringstream ss;
    ss << task->getName() << "_subtask";

    for(auto i : indices)
    {
        ss << "_" << i;
    }

    return ss.str();
}


TaskDescription::Ptr SubtaskImpl::getTask()
{
    return _task;
}


const Eigen::MatrixXd& SubtaskImpl::getWeight() const
{
    ::weightBigToSmall(_task->getWeight(),
                       _tmp_weight_small,
                       _indices);

    std::cout << _tmp_weight_small << std::endl;

    return _tmp_weight_small;
}

bool SubtaskImpl::setWeight(const Eigen::MatrixXd& value_small)
{
    _tmp_weight_big = _task->getWeight();
    ::weightSmallToBig(value_small, _tmp_weight_big, _indices);
    _task->setWeight(_tmp_weight_big);

    std::cout << _tmp_weight_big << std::endl;

    return true;
}

const std::vector<int>& SubtaskImpl::getSubtaskIndices() const
{
    return _indices;
}


void SubtaskImpl::reset()
{
    _task->reset();
}

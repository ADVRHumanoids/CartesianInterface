#ifndef __XBOT_CARTESIAN_PROBLEM_ANGULAR_MOMENTUM_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_ANGULAR_MOMENTUM_IMPL_H__

#include <cartesian_interface/problem/AngularMomentum.h>
#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {

/**
 * @brief Description of a postural (i.e. joint space) task
 *
 */
struct AngularMometumTaskImpl : public TaskDescriptionImpl,
        public virtual AngularMomentumTask
{

    CARTESIO_DECLARE_SMART_PTR(AngularMometumTaskImpl)


    AngularMometumTaskImpl(Context::ConstPtr context);

    AngularMometumTaskImpl(YAML::Node node, Context::ConstPtr contextl);


    void getAngularMomentumReference(Eigen::Vector3d &href) const override;

    void setAngularMomentumReference(const Eigen::Vector3d &href) override;

    void update(double time, double period) override;

    void setIndices(const std::vector<int>& value) override;

    void setDisabledJoints(const std::vector<std::string>& value) override;

    void reset() override;

private:

    Eigen::Vector3d _href;
    Eigen::Vector6d _momentum;

};


}
}

#endif

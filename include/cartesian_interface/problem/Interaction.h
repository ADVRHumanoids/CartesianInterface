#ifndef __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__
#define __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__

#include <cartesian_interface/problem/Cartesian.h>

namespace XBot { namespace Cartesian {

class InteractionTask : public virtual CartesianTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(InteractionTask)

    virtual const Eigen::Matrix6d& getStiffness() const = 0;
    virtual const Eigen::Matrix6d& getDamping() const = 0;
    virtual const Eigen::Matrix6d& getInertia() const = 0;
    virtual const Eigen::Vector6d& getForceReference() const = 0;
    virtual void getForceLimits(Eigen::Vector6d& fmin,
                                Eigen::Vector6d& fmax) const = 0;

    virtual void setStiffness(const Eigen::Matrix6d& k) = 0;
    virtual void setDamping(const Eigen::Matrix6d& d) = 0;
    virtual void setInertia(const Eigen::Matrix6d& m) = 0;
    virtual void setForceReference(const Eigen::Vector6d& f) = 0;
    virtual bool setForceLimits(const Eigen::Vector6d& fmin,
                                const Eigen::Vector6d& fmax)= 0;

};

class AdmittanceTask : public virtual InteractionTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(AdmittanceTask)

    virtual const Eigen::Vector6d& getForceDeadzone() const = 0;
    virtual const std::vector<std::string>& getForceEstimationChains() const = 0;

};

} }




#endif

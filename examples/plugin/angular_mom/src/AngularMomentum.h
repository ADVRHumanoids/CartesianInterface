#ifndef ANGULARMOMENTUM_H
#define ANGULARMOMENTUM_H

#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {

/**
 * @brief The AngularMomentum class models the abstract
 * interface for the task.
 *
 * NOTE: it is mandatory to inherit **virtually** from TaskDescription
 */
class AngularMomentum : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(AngularMomentum)

    /* Define the task API (pure virtual methods) */
    virtual Eigen::Vector3d getReference() const = 0;
    virtual void setReference(const Eigen::Vector3d& lref) = 0;

};

/**
 * @brief The AngularMomentumImpl class implements the abstract
 * interface
 */
class AngularMomentumImpl : public TaskDescriptionImpl,
                            public virtual AngularMomentum
{

public:

    CARTESIO_DECLARE_SMART_PTR(AngularMomentumImpl)

    /* The task implementation constructor signature must be
     * as follows */
    AngularMomentumImpl(YAML::Node task_node,
                        ModelInterface::ConstPtr model);

    /* Implement the task API */
    Eigen::Vector3d getReference() const override;
    void setReference(const Eigen::Vector3d& lref) override;

    /* Customize update, reset and log */
    void update(double time, double period) override;
    void reset() override;
    void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;

private:

    static constexpr double REF_TTL = 1.0;

    Eigen::Vector3d _lref;
    double _ref_timeout;
};

} }

#endif // ANGULARMOMENTUM_H

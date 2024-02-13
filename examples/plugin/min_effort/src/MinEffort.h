#ifndef MINEFFORT_H
#define MINEFFORT_H

#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {

/**
 * @brief The MinimumEffort class models the abstract
 * interface for the task.
 *
 * NOTE: it is mandatory to inherit **virtually** from TaskDescription
 */
class MinimumEffort : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(MinimumEffort)

    /* Define the task API (pure virtual methods) */

    virtual double getStepSize() const = 0;

};

/**
 * @brief The MinimumEffortImpl class implements the abstract
 * interface
 */
class MinimumEffortImpl : public TaskDescriptionImpl,
                          public virtual MinimumEffort
{

public:

    CARTESIO_DECLARE_SMART_PTR(MinimumEffortImpl)

    /* The task implementation constructor signature must be
     * as follows */
    MinimumEffortImpl(YAML::Node task_node,
                      Context::ConstPtr context);

    /* Implement the task API */

    double getStepSize() const override;

    /* Customize update, reset and log */
    void update(double time, double period) override;
    void reset() override;
    void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;

private:

    double _step_size;

};

} }

#endif // MinimumEffort_H

#ifndef MANIPULABILITY_H
#define MANIPULABILITY_H

#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {

/**
 * @brief The Manipulability class models the abstract
 * interface for the task.
 *
 * NOTE: it is mandatory to inherit **virtually** from TaskDescription
 */
class Manipulability : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(Manipulability)

    /* Define the task API (pure virtual methods) */

    virtual double getStepSize() const = 0;
    virtual const std::string& getBaseLink() const = 0;
    virtual const std::string& getDistalLink() const = 0;

};

/**
 * @brief The ManipulabilityImpl class implements the abstract
 * interface
 */
class ManipulabilityImpl : public TaskDescriptionImpl,
                          public virtual Manipulability
{

public:

    CARTESIO_DECLARE_SMART_PTR(ManipulabilityImpl)

    /* The task implementation constructor signature must be
     * as follows */
    ManipulabilityImpl(YAML::Node task_node,
                      Context::ConstPtr context);

    /* Implement the task API */

    double getStepSize() const override;
    const std::string& getBaseLink() const;
    const std::string& getDistalLink() const;

    /* Customize update, reset and log */
    void update(double time, double period) override;
    void reset() override;
    void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;

private:

    double _step_size;
    std::string _base_link;
    std::string _distal_link;

};

} }

#endif // Manipulability_H

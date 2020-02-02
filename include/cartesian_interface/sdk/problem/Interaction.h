#ifndef __XBOT_CARTESIAN_PROBLEM_INTERACTION_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_INTERACTION_IMPL_H__

#include <cartesian_interface/sdk/problem/Cartesian.h>
#include <cartesian_interface/problem/Interaction.h>

namespace XBot { namespace Cartesian {

class InteractionTaskImpl : public virtual InteractionTask,
        public CartesianTaskImpl

{

public:

    InteractionTaskImpl(ModelInterface::ConstPtr model,
                        std::string name,
                        std::string distal_link,
                        std::string base_link = "world");

    InteractionTaskImpl(ModelInterface::ConstPtr model,
                        std::string name,
                        std::string type,
                        std::string distal_link,
                        std::string base_link);

    InteractionTaskImpl(YAML::Node node,
                        ModelInterface::ConstPtr model);

    void update(double time, double period) override;
    void log(MatLogger::Ptr logger, bool init_logger, int buf_size) override;

    const Eigen::Matrix6d& getStiffness() const override;
    const Eigen::Matrix6d& getDamping() const override;
    const Eigen::Matrix6d& getInertia() const override;
    const Eigen::Vector6d& getForceReference() const override;
    void getForceLimits(Eigen::Vector6d& fmin,
                        Eigen::Vector6d& fmax) const override;

    void setStiffness(const Eigen::Matrix6d& k) override;
    void setDamping(const Eigen::Matrix6d& d) override;
    void setInertia(const Eigen::Matrix6d& m) override;
    void setForceReference(const Eigen::Vector6d& f) override;
    bool setForceLimits(const Eigen::Vector6d& fmin,
                        const Eigen::Vector6d& fmax) override;

private:

    static constexpr double REF_TTL = 0.3;

    Eigen::Matrix6d _k, _d, _m;
    Eigen::Vector6d _fref, _fmin, _fmax;
    double _ref_timeout;

};

class AdmittanceTaskImpl : public virtual AdmittanceTask,
        public InteractionTaskImpl
{

public:

    AdmittanceTaskImpl(YAML::Node node,
                       ModelInterface::ConstPtr model);

    const Eigen::Vector6d& getForceDeadzone() const override;
    const std::vector<std::string>& getForceEstimationChains() const override;

private:

    Eigen::Vector6d _dz;
    std::vector<std::string> _fest_chains;
};


} }




#endif

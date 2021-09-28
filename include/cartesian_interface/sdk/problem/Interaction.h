#ifndef __XBOT_CARTESIAN_PROBLEM_INTERACTION_IMPL_H__
#define __XBOT_CARTESIAN_PROBLEM_INTERACTION_IMPL_H__

#include <cartesian_interface/sdk/problem/Cartesian.h>
#include <cartesian_interface/problem/Interaction.h>

#include "XBotInterface/RobotInterface.h"

namespace XBot { namespace Cartesian {

class InteractionTaskImpl : public virtual InteractionTask,
        public CartesianTaskImpl

{

public:

    CARTESIO_DECLARE_SMART_PTR(InteractionTaskImpl)

    InteractionTaskImpl(Context::ConstPtr context,
                        std::string name,
                        std::string distal_link,
                        std::string base_link = "world");

    InteractionTaskImpl(Context::ConstPtr context,
                        std::string name,
                        std::string type,
                        std::string distal_link,
                        std::string base_link);

    InteractionTaskImpl(YAML::Node node,
                        Context::ConstPtr context);

    void update(double time, double period) override;
    void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;
    void reset() override;

    const Impedance & getImpedance () override;
	
    const Eigen::Vector6d& getForceReference() const override;
    void getForceLimits(Eigen::Vector6d& fmin,
                        Eigen::Vector6d& fmax) const override;

    bool setImpedance (const Impedance & impedance) override;
	
	void setForceReference(const Eigen::Vector6d& f) override;
    bool setForceLimits(const Eigen::Vector6d& fmin,
                        const Eigen::Vector6d& fmax) override;
	
	/* these methods are used to change the stiffness in a smooth way... */
	
	void  abortStiffnessTransition();
	bool  setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points);
	State getStiffnessState() const;

private:

    static constexpr double REF_TTL = 0.3;
	
	Impedance       _impedance;
	
    Eigen::Vector6d _fref, _fmin, _fmax;
    double          _ref_timeout;
	
	State                              _state;
	Interpolator<Eigen::Matrix6d>::Ptr _interpolator;

    Eigen::JacobiSVD<Eigen::MatrixXd> _JtKsvd;

    Eigen::MatrixXd _JtK, _J;
    Eigen::VectorXd _k, _q, _q_ref, _g, _tau, _tau_ref, _delta, _temp;

private:

    Eigen::Affine3d matchTorqueReference();

    void computeCurrentTorque(Eigen::VectorXd& tau);
    void computeEquivalentCartesianError(const Eigen::VectorXd& tau, Eigen::Vector6d& error);

    bool compareCurrentInteractionTorque(const Eigen::VectorXd& error, const Eigen::VectorXd& tau);
    bool computeEquivalentCartesianReference(const Eigen::Vector6d& error, Eigen::Affine3d& pose_reference);

    void computeJtK(Eigen::MatrixXd& jtk);
    void subtractGravity(Eigen::VectorXd& tau);

};

class AdmittanceTaskImpl : public virtual AdmittanceTask,
        public InteractionTaskImpl
{

public:

    AdmittanceTaskImpl(YAML::Node node,
                       Context::ConstPtr context);

    const Eigen::Vector6d& getForceDeadzone() const override;
    const std::vector<std::string>& getForceEstimationChains() const override;

private:

    Eigen::Vector6d _dz;
    std::vector<std::string> _fest_chains;
};


} }




#endif

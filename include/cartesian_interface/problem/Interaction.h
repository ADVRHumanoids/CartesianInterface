#ifndef __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__
#define __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__

#include <cartesian_interface/problem/Cartesian.h>
#include <cartesian_interface/trajectory/Interpolator.h>

#include <cartesian_interface/Enum.h>

namespace XBot { namespace Cartesian {

	
class Impedance
{
public:
	
	Eigen::Matrix6d stiffness;
	Eigen::Matrix6d damping  ;
	Eigen::Matrix6d mass     ;

    Impedance(const Eigen::Matrix6d& stiffness = Eigen::Matrix6d::Zero(),
              const Eigen::Matrix6d& damping = Eigen::Matrix6d::Zero(),
              const Eigen::Matrix6d& mass = Eigen::Matrix6d::Identity())
        : stiffness(stiffness), damping(damping), mass(mass) {}

};


class InteractionTask : public virtual CartesianTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(InteractionTask)

    virtual const Impedance & getImpedance () = 0;
	
    virtual const Eigen::Vector6d& getForceReference () const = 0;
    virtual void getForceLimits (Eigen::Vector6d& fmin,
                                 Eigen::Vector6d& fmax) const = 0;

    virtual bool setImpedance (const Impedance & impedance) = 0;
	
	
	virtual void setForceReference (const Eigen::Vector6d& f) = 0;
    virtual bool setForceLimits (const Eigen::Vector6d& fmin,
                                 const Eigen::Vector6d& fmax)= 0;
	
	virtual void  abortStiffnessTransition () = 0;
	virtual bool  setStiffnessTransition (const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points) = 0;
	virtual State getStiffnessState () const = 0;

    virtual const std::string& getImpedanceRefLink() const = 0;
    virtual bool setImpedanceRefLink(const std::string& new_impedance_ref_link) = 0;


};

class AdmittanceTask : public virtual InteractionTask
{

public:

    CARTESIO_DECLARE_SMART_PTR(AdmittanceTask)

    virtual const Eigen::Vector6d& getForceDeadzone() const = 0;
    virtual const std::vector<std::string>& getForceEstimationChains() const = 0;

};

} }



#endif // __XBOT_CARTESIAN_PROBLEM_INTERACTION_H__

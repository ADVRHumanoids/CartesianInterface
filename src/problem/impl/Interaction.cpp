#include "problem/Interaction.h"
#include "fmt/format.h"

#define TASK_SPACE_DIM 6
#define SAFETY_FACTOR 1.1
#define TRANS_ERROR_MAX 0.1
#define ROT_ERROR_MAX 1.0
#define MATCH_THRESHOLD 0.001

using namespace XBot::Cartesian;

InteractionTaskImpl::InteractionTaskImpl(YAML::Node task_node,
                                         Context::ConstPtr context):
    CartesianTaskImpl(task_node, context),
    _ref_timeout  (-1),
    _state        (State::Online),
    _fref         (Eigen::Vector6d::Zero()),
	_interpolator (new Interpolator<Eigen::Matrix6d>)
{
    int dof = _model->getJointNum();

    _k.setZero(dof); _q.setZero(dof); _q_ref.setZero(dof); _g.setZero(dof);
    _tau.setZero(dof); _tau_ref.setZero(dof); _delta.setZero(dof); _temp.setZero(dof);

    _JtK.setZero(dof, TASK_SPACE_DIM); _J.setZero(TASK_SPACE_DIM, dof);
    _JtKsvd.compute(_JtK, Eigen::ComputeThinU | Eigen::ComputeThinV);

    _robot = XBot::RobotInterface::getRobot(_model->getConfigOptions());

    std::vector<double> stiffness, damping, inertia, fmin, fmax;

    if(task_node["stiffness"])
    {
        stiffness = task_node["stiffness"].as<std::vector<double>>();
        if(stiffness.size() != 6)
        {
            throw std::runtime_error("'stiffness' field for interaction tasks requires six values");
        }
        _impedance.stiffness = Eigen::Vector6d::Map(stiffness.data()).asDiagonal();
    }
    else
    {
        throw std::runtime_error("'stiffness' field required for interaction tasks (six values)");
    }


    if(task_node["damping"])
    {
        damping = task_node["damping"].as<std::vector<double>>();
        if(damping.size() != 6)
        {
            throw std::runtime_error("'damping' field for interaction tasks requires six values");
        }
        _impedance.damping = Eigen::Vector6d::Map(damping.data()).asDiagonal();
    }
    else
    {
        throw std::runtime_error("'damping' field required for interaction tasks (six values)");
    }

    if(task_node["inertia"])
    {
        inertia = task_node["inertia"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'inertia' field for interaction tasks requires six values");
        }
        _impedance.mass = Eigen::Vector6d::Map(inertia.data()).asDiagonal();
    }
    else
    {
        _impedance.mass.setIdentity();
    }

    _fmin.setConstant(-1000.0);
    _fmax.setConstant(1000.0);
    if(task_node["force_min"])
    {
        fmin = task_node["force_min"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'force_min' field for interaction tasks requires six values");
        }
        _fmin = Eigen::Vector6d::Map(fmin.data());
    }

    if(task_node["force_max"])
    {
        fmax = task_node["force_max"].as<std::vector<double>>();
        if(inertia.size() != 6)
        {
            throw std::runtime_error("'force_max' field for interaction tasks requires six values");
        }
        _fmax = Eigen::Vector6d::Map(fmax.data());
    }

    if((_fmax - _fmin).minCoeff() < 0)
    {
        throw std::runtime_error("'force_max' must be component-wise greater-equal than 'force_min'");
    }

    if(_fmax.minCoeff() < 0 || _fmin.maxCoeff() > 0)
    {
        throw std::runtime_error("'force_max' must be >= 0 && 'force_min' must be <= 0");
    }

    if(_impedance.stiffness.minCoeff() < 0 || _impedance.damping.minCoeff() < 0 || _impedance.mass.minCoeff() < 0)
    {
        throw std::runtime_error("Negative values detected in interaction parameters");
    }

}

const Impedance & XBot::Cartesian::InteractionTaskImpl::getImpedance()
{
	return _impedance;
}

const Eigen::Vector6d& XBot::Cartesian::InteractionTaskImpl::getForceReference() const
{
    return _fref;
}

void XBot::Cartesian::InteractionTaskImpl::getForceLimits(Eigen::Vector6d& fmin,
                                                          Eigen::Vector6d& fmax) const
{
    fmin = _fmin;
    fmax = _fmax;
}

bool XBot::Cartesian::InteractionTaskImpl::setImpedance(const Impedance& impedance)
{
	if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in REACHING state \n",
                            getName().c_str());
        return false;
    }
    
    _impedance = impedance;
    return true;
}

void XBot::Cartesian::InteractionTaskImpl::setForceReference(const Eigen::Vector6d& f)
{
    _fref = f.cwiseMin(_fmax).cwiseMax(_fmin);
    _ref_timeout = getTime() + REF_TTL;
}

bool XBot::Cartesian::InteractionTaskImpl::setForceLimits(const Eigen::Vector6d& fmin,
                                                          const Eigen::Vector6d& fmax)
{
    if((fmax - fmin).minCoeff() < 0)
    {
        return false;
    }

    if(fmax.minCoeff() < 0 || fmin.maxCoeff() > 0)
    {
        return false;
    }

    _fmin = fmin;
    _fmax = fmax;
    _fref = _fref.cwiseMin(_fmax).cwiseMax(_fmin);

    return true;

}

bool XBot::Cartesian::InteractionTaskImpl::setStiffnessTransition(const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    _state = State::Reaching;
    
	_interpolator->clear();
	_interpolator->addWayPoint(getTime(), _impedance.stiffness);
	
    for(const auto& wp : way_points)
    {
		_interpolator->addWayPoint(wp, getTime());
    }
    
    _interpolator->compute();
	
    return true;
}

void XBot::Cartesian::InteractionTaskImpl::abortStiffnessTransition()
{
    if(_state == State::Reaching)
    {
        _state = State::Online;
        _interpolator->clear();
    }
}

void XBot::Cartesian::InteractionTaskImpl::update(double time, double period)
{
    CartesianTaskImpl::update(time, period);

    if(time > _ref_timeout) _fref.setZero();
	
	TaskDescriptionImpl::update(time, period);

    if (_state == State::Reaching)
    {
        _impedance.stiffness = _interpolator->evaluate(time);
		
		if (_interpolator->isTrajectoryEnded(time))
        {
			_state = State::Online;
        }
    }
}

State XBot::Cartesian::InteractionTaskImpl::getStiffnessState() const
{
	return _state;
}


void XBot::Cartesian::InteractionTaskImpl::log(MatLogger2::Ptr logger,
                                               bool init_logger,
                                               int buf_size)
{
    CartesianTaskImpl::log(logger, init_logger, buf_size);

    if(init_logger)
    {
        logger->create(getName() + "_fref", 6, 1, buf_size);
        logger->create(getName() + "_k", 6, 6, buf_size);
        logger->create(getName() + "_d", 6, 6, buf_size);
        logger->create(getName() + "_m", 6, 6, buf_size);

        return;
    }

    logger->add(getName() + "_fref", _fref               );
    logger->add(getName() + "_k",    _impedance.stiffness);
    logger->add(getName() + "_d",    _impedance.damping  );
    logger->add(getName() + "_m",    _impedance.mass     );

}


void XBot::Cartesian::InteractionTaskImpl::reset()
{
    // note: reset should be called only when the robot is not moving

    // provides torque reference continuity

//    _T = matchTorqueReference();

    _T = get_current_pose();

    _vel.setZero();
    _acc.setZero();

    _state = State::Online;

    reset_otg();
}


void XBot::Cartesian::InteractionTaskImpl::computeCurrentTorque(Eigen::VectorXd& tau)
{
    _robot->sense(false);

    _robot->getStiffness(_k);
    _robot->getJointPosition(_q);
    _robot->getPositionReference(_q_ref);

    _robot->getEffortReference(_tau_ref);

    tau.noalias() = _tau_ref;
    tau.noalias() += _k.asDiagonal() * (_q_ref - _q);
}


bool XBot::Cartesian::InteractionTaskImpl::compareCurrentInteractionTorque(const Eigen::VectorXd& error, const Eigen::VectorXd& tau)
{
    /* tau = g + JtKe */

    _temp = tau;

    computeJtK(_JtK);
    subtractGravity(_temp);

    _temp.noalias() -= _JtK * error;

    return _temp.norm() < MATCH_THRESHOLD;
}


void XBot::Cartesian::InteractionTaskImpl::computeJtK(Eigen::MatrixXd& jtk)
{
    if(_base_link == "world")
    {
        _model->getJacobian(_distal_link, _J);
    }
    else
    {
        _model->getJacobian(_distal_link, _base_link, _J);
    }

    jtk.noalias() = _J.transpose() * _impedance.stiffness;
}


void XBot::Cartesian::InteractionTaskImpl::subtractGravity(Eigen::VectorXd& tau)
{
    _model->computeGravityCompensation(_g);

    /**/

    tau -= _g;
}


void XBot::Cartesian::InteractionTaskImpl::computeEquivalentCartesianError(const Eigen::VectorXd& tau, Eigen::Vector6d& error)
{
    for (int i = 0; i < 3; i++)
    {
        _temp = tau;

        computeJtK(_JtK);
        subtractGravity(_temp);

        // least squares (minimum norm) solution:

        _JtKsvd.compute(_JtK, Eigen::ComputeThinU | Eigen::ComputeThinV);

        error = _JtKsvd.solve(_temp);

        double kt = error.head(3).norm() / TRANS_ERROR_MAX;
        double ko = error.tail(3).norm() / ROT_ERROR_MAX;

        if (kt <= 1 && ko <= 1)
        {
            break;
        }

        /* automatically correct impedance to respect error constraints */

        _impedance.stiffness = _impedance.stiffness * SAFETY_FACTOR * std::max(kt, ko);
    }
}


bool XBot::Cartesian::InteractionTaskImpl::computeEquivalentCartesianReference(const Eigen::Vector6d& error, Eigen::Affine3d& pose_reference)
{
    // error = sin(theta/2) * u

    Eigen::Vector3d translation_error = error.head(3);

    /* note thet in the control algorithm the computed orientation error is multiplied by 2.0 */

    Eigen::Vector3d orientation_error = error.tail(3) / 2.0;

    double angle = 0.; Eigen::Vector3d axis(0., 0., 1.);
    double norm = orientation_error.norm();

    if (norm > 1.0)
    {
        return false;
    }

    else if (norm > 0)
    {
        angle = asin(norm) * 2.0;
        axis = orientation_error / norm;
    }

    Eigen::Affine3d current_pose = get_current_pose();
    Eigen::AngleAxisd angle_axis(angle, axis);

    pose_reference.linear().noalias() = angle_axis.toRotationMatrix() * current_pose.linear();
    pose_reference.translation().noalias() = current_pose.translation() + translation_error;

    return true;
}


Eigen::Affine3d XBot::Cartesian::InteractionTaskImpl::matchTorqueReference()
{
    Eigen::Vector6d error;
    Eigen::Affine3d pose_reference;

    computeCurrentTorque(_tau);
    computeEquivalentCartesianError(_tau, error);

    if (computeEquivalentCartesianReference(error, pose_reference))
    {
        /* double check! */

        Eigen::Vector3d orientation_error;
        Eigen::Affine3d current_pose = get_current_pose();

        XBot::Utils::computeOrientationError(pose_reference.linear(), current_pose.linear(), orientation_error);

        error.head(3) = pose_reference.translation() - current_pose.translation();
        error.tail(3) = 2.0 * orientation_error;

        if (compareCurrentInteractionTorque(error, _tau))
        {
            return pose_reference;
        }
    }

    return get_current_pose();
}

//Eigen::Affine3d XBot::Cartesian::InteractionTaskImpl::matchTorqueReference(const Eigen::VectorXd& tau)
//{
//    _model->computeGravityCompensation(_g);

//    _delta = tau - _g;

//    if(_base_link == "world")
//    {
//        _model->getJacobian(_distal_link, _J);
//    }
//    else
//    {
//        _model->getJacobian(_distal_link, _base_link, _J);
//    }

//    _JtK.noalias() = _J.transpose() * _impedance.stiffness;

//    // least squares (minimum norm) solution:

//    _JtKsvd.compute(_JtK, Eigen::ComputeThinU | Eigen::ComputeThinV);

//    Eigen::Affine3d pose_reference;
//    Eigen::Vector6d error = _JtKsvd.solve(_delta);

//    if (getPoseReferenceFromError(error, pose_reference))
//    {
//        Eigen::Vector3d orientation_error;
//        auto current_pose = get_current_pose();

//        XBot::Utils::computeOrientationError(pose_reference.linear(), current_pose.linear(), orientation_error);

//        error.head(3) = pose_reference.translation() - current_pose.translation();
//        error.tail(3) = 2.0 * orientation_error;

//        double residual = (_JtK * error + _g - _tau).norm();

//        return pose_reference;
//    }

//    return get_current_pose();
//}


//bool XBot::Cartesian::InteractionTaskImpl::getPoseReferenceFromError(const Eigen::Vector6d& error, Eigen::Affine3d& pose_reference)
//{
//    // error = sin(theta/2) * u

//    Eigen::Vector3d translation_error = error.head(3);
//    Eigen::Vector3d orientation_error = error.tail(3) / 2.0; // !!!

//    double angle = 0.;
//    Eigen::Vector3d axis(0., 0., 1.);

//    double norm = orientation_error.norm();

//    if (norm > 1.0)
//    {
//        return false;
//    }

//    else if (norm > 0)
//    {
//        angle = asin(norm) * 2.0;
//        axis = orientation_error / norm;
//    }

//    Eigen::Affine3d current_pose = get_current_pose();
//    Eigen::AngleAxisd angle_axis(angle, axis);

//    pose_reference.linear().noalias() = angle_axis.toRotationMatrix() * current_pose.linear();
//    pose_reference.translation().noalias() = current_pose.translation() + translation_error;

//    return true;
//}


AdmittanceTaskImpl::AdmittanceTaskImpl(YAML::Node task_node,
                                       Context::ConstPtr context):
    InteractionTaskImpl(task_node, context)
{
    _dz.setZero();

    if(task_node["force_estimation_chains"])
    {
        _fest_chains = task_node["force_estimation_chains"].as<std::vector<std::string>>();
    }

    if(task_node["force_dead_zone"])
    {
        auto dz = task_node["force_dead_zone"].as<std::vector<double>>();
        if(dz.size() != 6)
        {
            throw std::runtime_error("'force_dead_zone' field for interaction tasks requires six values");
        }
        _dz = Eigen::Vector6d::Map(dz.data());

        if(_dz.minCoeff() < 0)
        {
            throw std::runtime_error("'force_dead_zone' values must be >= 0");
        }
    }
}

const Eigen::Vector6d& XBot::Cartesian::AdmittanceTaskImpl::getForceDeadzone() const
{
    return _dz;
}

const std::vector<std::string>& XBot::Cartesian::AdmittanceTaskImpl::getForceEstimationChains() const
{
    return _fest_chains;
}

#undef ROT_ERROR_MAX
#undef SAFETY_FACTOR
#undef TASK_SPACE_DIM
#undef TRANS_ERROR_MAX
#undef MATCH_THRESHOLD

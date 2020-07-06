#include <cartesian_interface/utils/estimation/TorqueEstimation.h>


using namespace XBot::Cartesian::Utils;

TorqueEstimation::TorqueEstimation(ModelInterface::ConstPtr model, 
				 bool momentum_based,
				 double obs_bw):
    _model(model),
//     _ndofs(0),
    _momentum_based(momentum_based),
    _k_obs(2.0 * M_PI * obs_bw),
    _logger(XBot::MatLogger::getLogger("/tmp/torque_estimation_log"))
{
    _period=0.0;
    init_momentum_obs();
    log(_logger);
}

void TorqueEstimation::update(double period)
{
    compute_residuals(period);
    
    log(_logger);
}

void XBot::Cartesian::Utils::TorqueEstimation::log(MatLogger::Ptr logger) const
{
    logger->add("fest_tau", _tau);
    logger->add("fest_g", _g);
    logger->add("fest_res", _y);
    logger->add("fest_static_res", _y_static);
}

bool TorqueEstimation::get_residuals(Eigen::VectorXd &res) const
{
    res = _y;
    
    return true;
}

bool TorqueEstimation::get_static_residuals(Eigen::VectorXd &static_res) const
{
    static_res = _y_static;
    
    return true;
}

void TorqueEstimation::compute_residuals(double period)
{
    _period = period;
    
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);
    
    /* Check for torque spikes */
    const double MAX_ALLOWED_TORQUE = 300.0;
    if((_tau.array().abs() < MAX_ALLOWED_TORQUE).all())
    {
        _y_static = _g - _tau;
    }
    
    if(_momentum_based){
	
	_model->getJointVelocity(_qdot);
	
	/* Observer */
	_model->getInertiaMatrix(_M);
	_p1 = _M * _qdot;
	
    /* check to avoid dividing by zero. XbotCore first control loops can return period of zero duration*/
    if(period > 0){
        _Mdot.noalias() = (_M - _M_old) / period;
    }
    else {
        _Mdot.setZero();
    }
	_M_old.noalias() = _M;
	_model->computeNonlinearTerm(_h);
	_model->computeGravityCompensation(_g);
	_coriolis.noalias() = _h - _g;
	_model->getJointEffort(_tau);
	_p2.noalias() += (_tau + (_Mdot * _qdot - _coriolis) - _g + _y) * period;
	_y = _k_obs*(_p1 - _p2 - _p0);
    }
    else {
	_y = _y_static;
    }
}

void TorqueEstimation::init_momentum_obs()
{
    _p1.setZero(_model->getJointNum());
    _p2.setZero(_model->getJointNum());
    _y.setZero(_model->getJointNum());
    _y_static.setZero(_model->getJointNum());
    _coriolis.setZero(_model->getJointNum());
    _h.setZero(_model->getJointNum());
    
    _model->getInertiaMatrix(_M);
    _model->getJointPosition(_q);
    _model->getJointVelocity(_qdot);
    _p0 = _M * _qdot;
    
    _M_old = _M;
    
    // initialize size 
    _tau.setZero(_model->getJointNum());
    _g.setZero(_model->getJointNum());
    
    _Mdot = _M;
}

TorqueEstimation::~TorqueEstimation()
{
    _logger->flush();
}



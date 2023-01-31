#include <cartesian_interface/utils/estimation/ForceEstimation.h>

using namespace XBot::Cartesian::Utils;

const double ForceEstimation::DEFAULT_SVD_THRESHOLD = 0.05;

NumInt::NumInt(int n_jnts, double dt, double T_horizon)
    :_n_jnts{n_jnts}, _dt{dt}, _T_horizon{T_horizon}
{
    _n_intervals = std::round(_T_horizon / _dt);

    _n_samples = _n_intervals + 1;

    _window_data = Eigen::MatrixXd::Zero(_n_jnts, _n_samples);

}

void NumInt::add_sample(Eigen::VectorXd sample)
{
  int sample_size = sample.size();

  if(sample_size != _n_jnts)
  {
      std::string exception = std::string("NumInt::add_sample(): Trying to add a sample of size ") +
                              std::to_string(sample_size) + std::string(", which is different from ") +
                              std::to_string(_n_jnts) + std::string(", (number of joints) \n");

      throw std::invalid_argument(exception);
  }

  // shifting data to the right (discarting most remote sample, which is the
  // one on the extreme right)
  for (int i = _n_samples - 1; i > 0; i--)
  {
     _window_data.block(0, i, _n_jnts, 1) = _window_data.block(0, i - 1, _n_jnts, 1);

  }

  _window_data.block(0, 0 , _n_jnts, 1) = sample; // assign most recent sample

}

void NumInt::get(Eigen::VectorXd& sample_integral)
{
    sample_integral = Eigen::VectorXd::Zero(_n_jnts);

    for(int i = _n_intervals; i > 0; i--)
    { // we integrate all the data in the window
        sample_integral = sample_integral +
                ( _window_data.block(0, i, _n_jnts, 1) +
                  _window_data.block(0, i - 1, _n_jnts, 1) ) / 2.0 * _dt;
    }
}

ForceEstimation::ForceEstimation(ModelInterface::ConstPtr model, 
                                 double svd_threshold):
    _model(model),
    _ndofs(0)
{
    _svd.setThreshold(svd_threshold);
}


XBot::ForceTorqueSensor::ConstPtr ForceEstimation::add_link(std::string name, 
                                                            std::vector<int> dofs,
                                                            std::vector<std::string> chains)
{
    // check link exists
    auto urdf_link = _model->getUrdf().getLink(name);
    
    if(!urdf_link)
    {
        throw std::invalid_argument("Invalid link '" + name + "'");
    }
    
    // wrench dofs if not provided
    if(dofs.size() == 0)
    {
        dofs = {0, 1, 2, 3, 4, 5};
    }
    
    // chains to use for estimation if not provided
    if(chains.size() == 0)
    {
        chains = _model->getChainNames();
    }
    
    // check dofs are valid
    auto it = std::find_if(dofs.begin(), 
                           dofs.end(), 
                           [](int dof){ return dof >= 6 || dof < 0; });
    if(it != dofs.end())
    {
        throw std::invalid_argument("Dofs must be >= 0 && < 6");
    }
    
    // add torque sensing dofs for the requested chains
    std::vector<int> meas_dofs;
    for(auto ch : chains)
    {
        if(!_model->hasChain(ch))
        {
            throw std::invalid_argument("Invalid chain '" + ch + "'");
        }
        
        for(int id : _model->chain(ch).getJointIds())
        {
            meas_dofs.push_back(_model->getDofIndex(id));
        }
    }
    
    _meas_idx.insert(meas_dofs.begin(), meas_dofs.end());

    // remove ignored ids
    for(int ignid : _ignore_idx)
    {
        _meas_idx.erase(ignid);
    }
    
    // make virtual sensor and task info struct
    TaskInfo t;
    t.link_name = name;
    static int id = -1;
    t.sensor = std::make_shared<ForceTorqueSensor>(urdf_link, id--);
    t.dofs = dofs;
    
    _tasks.push_back(t);
    
    _ndofs += dofs.size();
    
    return t.sensor;
    
}

void ForceEstimation::setIgnoredJoint(const std::string &jname)
{
    int idx = _model->getDofIndex(jname);
    if(idx < 0)
    {
        throw std::invalid_argument("invalid joint '" + jname + "'");
    }

    // add to ignored ids set
    _ignore_idx.insert(idx);

    // remove from meas ids set
    _meas_idx.erase(idx);
}


void ForceEstimation::compute_A_b()
{
    _Jtot.setZero(_ndofs, _model->getJointNum());
    _Jtmp.setZero(6, _model->getJointNum());
    _b.setZero(_meas_idx.size());
    _A.setZero(_meas_idx.size(), _ndofs);
    
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        _model->getJacobian(t.link_name, _Jtmp);
        for(int i : t.dofs)
        {
            _Jtot.row(dof_idx++) = _Jtmp.row(i);
        }
    }
    
    compute_residual(_y);
    
    int idx = 0;
    for(int i : _meas_idx)
    {
        _b(idx) = _y(i);
        _A.row(idx) = _Jtot.col(i);
        idx++;
    }
    
}

void ForceEstimation::solve()
{
    _svd.compute(_A, Eigen::ComputeThinU|Eigen::ComputeThinV);
    _sol = _svd.solve(_b);

//    _qr.compute(_A);
//    _sol = _qr.solve(_b);
}

void ForceEstimation::compute_residual(Eigen::VectorXd& res)
{
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);

    res = _g - _tau;

}


void ForceEstimation::update()
{
    compute_A_b();
    
    solve();
    
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        Eigen::Vector6d wrench;
        wrench.setZero();
        
        for(int i : t.dofs)
        {
            wrench(i) = _sol(dof_idx++);
        }
        
        Eigen::Matrix3d sensor_R_w;
        _model->getOrientation(t.link_name, sensor_R_w);
        sensor_R_w.transposeInPlace();
        
        wrench.head<3>() = sensor_R_w * wrench.head<3>();
        wrench.tail<3>() = sensor_R_w * wrench.tail<3>();
        
        t.sensor->setWrench(wrench, 0.0);
        
    }
}

void XBot::Cartesian::Utils::ForceEstimation::log(MatLogger2::Ptr logger) const
{
    for(const TaskInfo& t : _tasks)
    {
        Eigen::Vector6d wrench;
        t.sensor->getWrench(wrench);
        
        Eigen::Matrix3d w_R_s;
        _model->getOrientation(t.link_name, w_R_s);
        
        wrench.head<3>() = w_R_s * wrench.head<3>();
        wrench.tail<3>() = w_R_s * wrench.tail<3>();
        
        logger->add(t.link_name + "_f_est", wrench);
    }
    
    logger->add("fest_A", _A);
    logger->add("fest_b", _b);
    logger->add("fest_tau", _tau);
    logger->add("fest_g", _g);
}


ForceEstimationMomentumBased::ForceEstimationMomentumBased(XBot::ModelInterface::ConstPtr model,
                                                           double rate,
                                                           double svd_threshold,
                                                           double obs_bw):
    ForceEstimation(model, svd_threshold),
    _k_obs(2.0 * M_PI * obs_bw),
    _rate(rate)
{
    init_momentum_obs();
}

bool ForceEstimationMomentumBased::getResiduals(Eigen::VectorXd& res) const
{
    res = _y;
    return true;
}

void ForceEstimationMomentumBased::compute_residual(Eigen::VectorXd& res)
{
    _model->getJointVelocity(_qdot_k);
    _model->getJointEffort(_tau_k);
    _model->getInertiaMatrix(_M_k);
    _model->computeNonlinearTerm(_h_k);

    _p_k = _M_k * _qdot_k;
    _Mdot_k = (_M_k - _M_km1) * _rate;

    _to_be_integrated = _h_k + _tau_k - _Mdot_k * _qdot_k;

    _integrator.add_sample(_to_be_integrated);
    _integrator.get(_integral);

    _y = _y_km1 * _c1 / _c2 +
            _k_obs / _c2 * (_p_k - _p_km1 + _integral);

    _y_km1 = _y;
    _p_km1 = _p_k;
    _M_km1 = _M_k;

    getResiduals(res);
    
}

void ForceEstimationMomentumBased::init_momentum_obs()
{
    _y.setZero(_model->getJointNum());
    _y_km1.setZero(_model->getJointNum());
    _tau_k.setZero(_model->getJointNum());
    _h_k.setZero(_model->getJointNum());
    _p_km1.setZero(_model->getJointNum());
    _p_k.setZero(_model->getJointNum());

    _b.setZero(_model->getJointNum());

    _model->getInertiaMatrix(_M_k);
    _model->getJointVelocity(_qdot_k);

    _integrator = NumInt(_y.size(), 1.0/_rate, 1.0/_rate);

    _M_km1 = _M_k;

    _c1 = 1 - 1.0/(2 * _rate) * _k_obs;
    _c2 = 1 + 1.0/(2 * _rate) * _k_obs;
}

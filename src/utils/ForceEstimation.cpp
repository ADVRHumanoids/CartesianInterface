#include <cartesian_interface/utils/estimation/ForceEstimation.h>


using namespace XBot::Cartesian::Utils;

ForceEstimation::ForceEstimation(ModelInterface::ConstPtr model, 
                                 double svd_threshold):
    _model(model),
    _ndofs(0),
    _svd_th(svd_threshold)
//     _logger(XBot::MatLogger::getLogger("/tmp/force_estimation_log"))
{    
    
}

ForceEstimation::~ForceEstimation()
{
//     _logger->flush();
}

void ForceEstimation::allocate_workspace() 
{
    _Jtot.setZero(_ndofs, _model->getJointNum());
    _Jtmp.setZero(6, _model->getJointNum());    
    _A.setZero(_meas_idx.size(), _ndofs);
    _b.setZero(_meas_idx.size());
    _sol.setZero(_ndofs);
    _svd = LapackSvd(_meas_idx.size(), _ndofs);
    _svd.setThreshold(_svd_th);
    _svd.compute(_A);
}

XBot::ForceTorqueSensor::ConstPtr ForceEstimation::add_link(std::string name, 
                                                            std::vector<int> dofs,
                                                            std::vector<std::string> chains)
{
    
    auto urdf_link = _model->getUrdf().getLink(name);
    
    if(!urdf_link)
    {
        throw std::invalid_argument("Invalid link '" + name + "'");
    }
    
    if(dofs.size() == 0)
    {
        dofs = {0, 1, 2, 3, 4, 5};
    }
    
    if(chains.size() == 0)
    {
        chains = _model->getChainNames();
    }
    
    auto it = std::find_if(dofs.begin(), 
                           dofs.end(), 
                           [](int dof){ return dof >= 6 || dof < 0; });
    if(it != dofs.end())
    {
        throw std::invalid_argument("Dofs must be >= 0 && < 6");
    }
    
    std::cout << "Adding link " << name << " to Force estimation:\n";
    std::cout << "Dofs to estimate: \n";
    for(auto d : dofs)
        std::cout << d << "\n";
    std::cout << "Chains to use: " << "\n";
    for(auto ch : chains)
        std::cout << ch << "\n";
    std::cout << std::endl;
    
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
    TaskInfo t;
    t.link_name = name;
    static int id = -1;
    t.sensor = std::make_shared<ForceTorqueSensor>(urdf_link, id--);
    t.dofs = dofs;
    t.wrench.setZero();
    t.s_R_w.setZero();
    
    _tasks.push_back(t);
    _ndofs += dofs.size();
    
    std::cout << "Force estimation using joints:\n";
    std::for_each(_meas_idx.begin(), _meas_idx.end(), 
                  [this](int i){ std::cout << _model->getEnabledJointNames().at(i) << "\n"; });
    std::cout << std::endl;
    
    allocate_workspace();
    
    return t.sensor;
    
}

void ForceEstimation::compute_residual()
{
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);
    
    /* Check for torque spikes */
    const double MAX_ALLOWED_TORQUE = 300.0;
    if((_tau.array().abs() < MAX_ALLOWED_TORQUE).all())
    {
        _y = _g - _tau;
    }
}

void ForceEstimation::compute_A_b()
{   
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        _model->getJacobian(t.link_name, _Jtmp);
        for(int i : t.dofs)
        {
            _Jtot.row(dof_idx++).noalias() = _Jtmp.row(i);
        }
    }

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
    _svd.compute(_A);
    _svd.solve2(_b, _sol);
}

void ForceEstimation::update()
{
    compute_residual();
    
    compute_A_b();
    
    solve();
    
    int dof_idx = 0;
    for(TaskInfo& t : _tasks)
    {
        t.wrench.setZero();
        
        for(int i : t.dofs)
        {
            t.wrench(i) = _sol(dof_idx++);
        }
        
        _model->getOrientation(t.link_name, t.s_R_w);
        t.s_R_w.transposeInPlace();
        
        t.wrench.head<3>() = t.s_R_w * t.wrench.head<3>();
        t.wrench.tail<3>() = t.s_R_w * t.wrench.tail<3>();
        
        t.sensor->setWrench(t.wrench, 0.0);
        t.sensor->getWrench(_local_wrench);
        
        t.wrench.head<3>() = t.s_R_w.transpose() * t.wrench.head<3>();
        t.wrench.tail<3>() = t.s_R_w.transpose() * t.wrench.tail<3>();
        
    }
    
//     log(_logger);
}

void XBot::Cartesian::Utils::ForceEstimation::log(MatLogger::Ptr logger) const
{
    for(const TaskInfo& t : _tasks)
    {
        logger->add(t.link_name + "_f_est", t.wrench);
        logger->add(t.link_name + "_f_est_local", _local_wrench);
    }
    
    // logger->add("fest_A", _A);
    // logger->add("fest_b", _b);
    logger->add("fest_sol", _sol);
    logger->add("fest_tau", _tau);
    logger->add("fest_g", _g);
    logger->add("fest_res", _y);
}




ForceEstimationMomentumBased::ForceEstimationMomentumBased(XBot::ModelInterface::ConstPtr model, 
                                                            double rate, 
                                                            double obs_bw,
                                                            double svd_threshold) : 
    ForceEstimation ( model, svd_threshold ),
    _rate(rate),
    _k_obs(2.0 * M_PI * obs_bw)
{    
    init_momentum_obs();
}

void ForceEstimationMomentumBased::init_momentum_obs()
{
    _p1.setZero(_model->getJointNum());
    _p2.setZero(_model->getJointNum());
    _y.setZero(_model->getJointNum());
    _y_static.setZero(_model->getJointNum());
    _coriolis.setZero(_model->getJointNum());
    _h.setZero(_model->getJointNum());
    
    _model->getInertiaMatrix(_M);
    _model->getJointVelocity(_qdot);
    _p0 = _M * _qdot;
    
    _M_old = _M;
    
    _tau.setZero(_model->getJointNum());
    _g.setZero(_model->getJointNum());
    
    _Mdot = _M;
}

void ForceEstimationMomentumBased::compute_residual() 
{
    _model->getJointVelocity(_qdot);
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);
    
    /* Check for torque spikes */
    const double MAX_ALLOWED_TORQUE = 300.0;
    if((_tau.array().abs() < MAX_ALLOWED_TORQUE).all())
    {
        _y_static = _g - _tau;
    }
    
    /* Observer */
    _model->getInertiaMatrix(_M);
    _p1.noalias() = _M * _qdot;
    
    _Mdot.noalias() = (_M - _M_old) * _rate;
    _M_old.noalias() = _M;
    _model->computeNonlinearTerm(_h);
    _coriolis.noalias() = _h - _g;
    _model->getJointEffort(_tau);
    _p2.noalias() += (_tau + (_Mdot * _qdot - _coriolis) - _g + _y) / _rate;
    _h.noalias() = _Mdot * _qdot - _coriolis;
    
    _y = _k_obs*(_p1 - _p2 - _p0);
}

bool ForceEstimationMomentumBased::get_residual(Eigen::VectorXd &res) const
{
    res = _y;   
    return true;
}

bool ForceEstimationMomentumBased::get_static_residual(Eigen::VectorXd &static_res) const
{
    static_res = _y_static;
    return true;
}

void ForceEstimationMomentumBased::log(MatLogger::Ptr logger) const
{
    XBot::Cartesian::Utils::ForceEstimation::log(logger);
    logger->add("fest_static_res", _y_static);
    logger->add("fest_Mdot", _Mdot);
//     logger->add("fest_M", _M);
    logger->add("fest_coriolis", _coriolis);
    logger->add("fest_h", _h);
    logger->add("fest_p1", _p1);
    logger->add("fest_p2", _p2);
    logger->add("fest_p0", _p0);
    logger->add("fest_qdot", _qdot);

}


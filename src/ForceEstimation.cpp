#include <cartesian_interface/utils/estimation/ForceEstimation.h>


using namespace XBot::Cartesian::Utils;

ForceEstimation::ForceEstimation(ModelInterface::ConstPtr model):
    _model(model),
    _ndofs(0)
{
    
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
    
    _tasks.push_back(t);
    
    _ndofs += dofs.size();
    
    return t.sensor;
    
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
    
    _model->getJointEffort(_tau);
    _model->computeGravityCompensation(_g);
    
    _y = _g - _tau;
    
    for(int i : _meas_idx)
    {
        _b(i) = _y(i);
        _A.row(i) = _Jtmp.col(i);
    }
    
}

void ForceEstimation::solve()
{
    _sol = _A.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(_b);
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
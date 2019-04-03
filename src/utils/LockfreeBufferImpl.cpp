#include <cartesian_interface/utils/LockfreeBufferImpl.h>

#define NOT_IMPL throw std::runtime_error(__PRETTY_FUNCTION__ + std::string(" not implemented"))

namespace XBot { namespace Cartesian {

LockfreeBufferImpl::LockfreeBufferImpl(CartesianInterface const * ci, ModelInterface::Ptr model):
    _model(model),
    _q_tmp(model->getJointNum()),
    _q_tmp_read(model->getJointNum()),
    _tasklist(ci->getTaskList())
{
    _model_state_queue.reset(_q_tmp);
    
    _task_tmp.base_frame.reserve(40);
    
    for(auto t : ci->getTaskList())
    {
        _taskstate_map[t] = _task_tmp;
        _taskstate_queue_map[t];
        
        if(ci->getTaskInterface(t) == TaskInterface::Interaction)
        {
            _inter_taskstate_map[t];
            _inter_taskstate_queue_map[t];
        }
    }
    
}

void LockfreeBufferImpl::callAvailable(CartesianInterface* ci)
{
    _call_queue.consume_all
    (
        [ci](CallbackType& f)
        {
            if(ci)
            {
                f(ci);
            }
        }
    );
}

void LockfreeBufferImpl::pushState(CartesianInterface const * ci, ModelInterface const * model)
{
    for(auto& pair : _taskstate_queue_map)
    {
        TaskState& t = _task_tmp;
        t.base_frame = ci->getBaseLink(pair.first);
        ci->getPoseReference(pair.first, t.Totg, &(t.vel), &(t.acc));
        ci->getPoseReferenceRaw(pair.first, t.T, &(t.vel), &(t.acc));
        ci->getVelocityLimits(pair.first, t.maxvel.first, t.maxvel.second);
        ci->getAccelerationLimits(pair.first, t.maxacc.first, t.maxacc.second);
        t.control_type = ci->getControlMode(pair.first);
        t.state = ci->getTaskState(pair.first);
        
        pair.second.push(t);
    }
    
    
    for(auto& pair : _inter_taskstate_queue_map)
    {
        InteractionTaskState t;
        ci->getDesiredInteraction(pair.first, 
                                  t.force, 
                                  t.k, 
                                  t.d);
        
        pair.second.push(t);
    }
    
    model->getJointPosition(_q_tmp);
    _model_state_queue.push(_q_tmp);
}

void LockfreeBufferImpl::updateState()
{
    for(auto& pair : _taskstate_map)
    {
        auto& queue = _taskstate_queue_map.at(pair.first);
        while(queue.pop(pair.second));
            
    }
    
    for(auto& pair : _inter_taskstate_map)
    {
        auto& queue = _inter_taskstate_queue_map.at(pair.first);
        while(queue.pop(pair.second));
            
    }
    
    while(_model_state_queue.pop(_q_tmp_read));
    _model->setJointPosition(_q_tmp_read);
    _model->update();
}

bool LockfreeBufferImpl::setPoseReference(const std::string& end_effector, 
                                          const Eigen::Affine3d& base_T_ref)
{
    CallbackType f = std::bind(&CartesianInterface::setPoseReference, 
        std::placeholders::_1, 
        end_effector, 
        base_T_ref);
    
    _call_queue.push(f);
    
}

bool LockfreeBufferImpl::setBaseLink(const std::string& ee_name, 
                                     const std::string& new_base_link)
{
    CallbackType f = std::bind(&CartesianInterface::setBaseLink, 
        std::placeholders::_1, 
        ee_name, new_base_link);
    
    return _call_queue.push(f);
}

bool LockfreeBufferImpl::setControlMode(const std::string& ee_name, ControlType ctrl_type)
{
    CallbackType f = std::bind(&CartesianInterface::setControlMode, 
        std::placeholders::_1, 
        ee_name, ctrl_type);
    
    return _call_queue.push(f);
}

bool LockfreeBufferImpl::setWayPoints(const std::string& end_effector, 
                                      const Trajectory::WayPointVector& way_points)
{
    CallbackType f = std::bind(&CartesianInterface::setWayPoints, 
        std::placeholders::_1, 
        end_effector, way_points);
    
    return _call_queue.push(f);
}

bool LockfreeBufferImpl::abort(const std::string& end_effector)
{
    CallbackType f = std::bind(&CartesianInterface::abort, 
        std::placeholders::_1, 
        end_effector);
    
    return _call_queue.push(f);
}

void LockfreeBufferImpl::getAccelerationLimits(const std::string& ee_name, 
                                               double& max_acc_lin, 
                                               double& max_acc_ang) const
{
    const auto& task = _taskstate_map.at(ee_name);
    max_acc_lin = task.maxacc.first;
    max_acc_ang = task.maxacc.second;
}

const std::string& LockfreeBufferImpl::getBaseLink(const std::string& ee_name) const
{
    return _taskstate_map.at(ee_name).base_frame;
}

bool LockfreeBufferImpl::getComPositionReference(Eigen::Vector3d& w_com_ref, 
                                                 Eigen::Vector3d* base_vel_ref,
                                                 Eigen::Vector3d* base_acc_ref) const
{
    NOT_IMPL;
}

ControlType LockfreeBufferImpl::getControlMode(const std::string& ee_name) const
{
    return _taskstate_map.at(ee_name).control_type;
}

bool LockfreeBufferImpl::getCurrentPose(const std::string& end_effector, 
                                        Eigen::Affine3d& base_T_ee) const
{
    const auto& task = _taskstate_map.at(end_effector);
    
    if(task.base_frame == "world")
    {
        if(end_effector == "com")
        {
            Eigen::Vector3d com;
            base_T_ee.setIdentity();
            _model->getCOM(com);
            base_T_ee.translation() = com;
        }
        else
        {
            _model->getPose(end_effector, base_T_ee);
        }
    }
    else
    {
        _model->getPose(end_effector, task.base_frame,  base_T_ee);
    }
    
    return true;
}

bool LockfreeBufferImpl::getPoseReference(const std::string& end_effector, 
                                          Eigen::Affine3d& base_T_ref, 
                                          Eigen::Vector6d* base_vel_ref, 
                                          Eigen::Vector6d* base_acc_ref) const
{
    base_T_ref = _taskstate_map.at(end_effector).Totg;
    return true;
}

bool LockfreeBufferImpl::getPoseReferenceRaw(const std::string& end_effector,
                                             Eigen::Affine3d& base_T_ref,
                                             Eigen::Vector6d* base_vel_ref,
                                             Eigen::Vector6d* base_acc_ref) const 
{
    base_T_ref = _taskstate_map.at(end_effector).T;
    return true;
}

bool LockfreeBufferImpl::getPoseTarget(const std::string& end_effector, 
                                       Eigen::Affine3d& base_T_ref) const
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::getReferencePosture(XBot::JointNameMap& qref) const
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::getReferencePosture(Eigen::VectorXd& qref) const
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
{
    NOT_IMPL;
}

const std::vector< std::string >& LockfreeBufferImpl::getTaskList() const
{
    return _tasklist;
}

State LockfreeBufferImpl::getTaskState(const std::string& end_effector) const
{
    return _taskstate_map.at(end_effector).state;
}

void LockfreeBufferImpl::getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const
{
    const auto& task = _taskstate_map.at(ee_name);
    max_vel_lin = task.maxvel.first;
    max_vel_ang = task.maxvel.second;
}

bool LockfreeBufferImpl::reset(double time)
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    CallbackType f = std::bind(&CartesianInterface::resetWorld, 
        std::placeholders::_1, 
        w_T_new_world);
    
    return _call_queue.push(f);
}

void LockfreeBufferImpl::setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang)
{
    CallbackType f = std::bind(&CartesianInterface::setAccelerationLimits, 
        std::placeholders::_1, 
        ee_name, max_acc_lin, max_acc_ang);
    
    _call_queue.push(f);
}

bool LockfreeBufferImpl::update(double time, double period)
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::setComPositionReference(const Eigen::Vector3d& base_com_ref)
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::setReferencePosture(const JointNameMap& qref)
{
    CallbackType f = std::bind(&CartesianInterface::setReferencePosture, 
        std::placeholders::_1, 
        qref);
    
    _call_queue.push(f);
}

bool XBot::Cartesian::LockfreeBufferImpl::setComVelocityReference(const Eigen::Vector3d& base_vel_ref)
{
    NOT_IMPL;
}

bool XBot::Cartesian::LockfreeBufferImpl::setVelocityReference(const std::string& end_effector, 
                                                               const Eigen::Vector6d& base_vel_ref)
{
    CallbackType f = std::bind(&CartesianInterface::setVelocityReference, 
        std::placeholders::_1, 
        end_effector, base_vel_ref);
    
    _call_queue.push(f);
}


bool LockfreeBufferImpl::setPoseReferenceRaw(const std::string& end_effector, 
                                             const Eigen::Affine3d& base_T_ref)
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time)
{
    NOT_IMPL;
}

bool LockfreeBufferImpl::setTargetPose(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, double time)
{
    NOT_IMPL;
}

void LockfreeBufferImpl::setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang)
{
    CallbackType f = std::bind(&CartesianInterface::setVelocityLimits, 
        std::placeholders::_1, 
        ee_name, max_vel_lin, max_vel_ang);
    
    _call_queue.push(f);
}



bool LockfreeBufferImpl::getDesiredInteraction(const std::string& end_effector,
                                                                Eigen::Vector6d& force, 
                                                                Eigen::Matrix6d& stiffness, 
                                                                Eigen::Matrix6d& damping) const
{
    const InteractionTaskState& t = _inter_taskstate_map.at(end_effector);
    force = t.force;
    stiffness = t.k;
    damping = t.d;
    
    return true;
}


TaskInterface LockfreeBufferImpl::getTaskInterface(const std::string& end_effector) const
{
    auto it = _taskstate_map.find(end_effector);
    
    if(it == _taskstate_map.end())
    {
        return TaskInterface::None;
    }
    
    auto jt = _inter_taskstate_map.find(end_effector);
    
    if(jt == _inter_taskstate_map.end())
    {
        return TaskInterface::Cartesian;
    }
    else
    {
        return TaskInterface::Interaction;
    }
    
}

bool LockfreeBufferImpl::setDesiredDamping(const std::string& end_effector, 
                                                            const Eigen::Matrix6d& d)
{
    CallbackType f = std::bind(&CartesianInterface::setDesiredDamping, 
        std::placeholders::_1, 
        end_effector, d);
    
    return _call_queue.push(f);
}

bool LockfreeBufferImpl::setDesiredStiffness(const std::string& end_effector,
                                                              const Eigen::Matrix6d& k)
{
    CallbackType f = std::bind(&CartesianInterface::setDesiredStiffness, 
        std::placeholders::_1, 
        end_effector, k);
    
    return _call_queue.push(f);
}

bool LockfreeBufferImpl::setForceReference(const std::string& end_effector, 
                                                            const Eigen::Vector6d& force)
{
    CallbackType f = std::bind(&CartesianInterface::setForceReference, 
        std::placeholders::_1, 
        end_effector, force);
    
    return _call_queue.push(f);
}



}}

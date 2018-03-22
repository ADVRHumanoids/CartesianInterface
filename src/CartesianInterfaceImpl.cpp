#include <cartesian_interface/CartesianInterfaceImpl.h>

using namespace XBot::Cartesian;

std::string CartesianInterface::ControlTypeAsString(CartesianInterface::ControlType ctrl)
{
    switch(ctrl)
    {
        case ControlType::Disabled:
            return "Disabled";
            break;
            
        case ControlType::Position:
            return "Position";
            break;
            
        case ControlType::Velocity:
            return "Velocity";
            break;
            
        default:
            return "Invalid control type";
    }
}


std::string CartesianInterface::StateAsString(CartesianInterface::State ctrl)
{
    switch(ctrl)
    {
        case State::Reaching:
            return "Reaching";
            break;
            
        case State::Online:
            return "Online";
            break;
            
        default:
            return "Invalid state";
    }
}

bool XBot::Cartesian::CartesianInterfaceImpl::setBaseLink(const std::string& ee_name, 
                                                          const std::string& new_base_link)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return false;
    }
    
    Eigen::Affine3d new_T_old;
    
    if( !_model->getPose(task->base_frame, new_base_link, new_T_old) )
    {
        XBot::Logger::error("New base link %s in not defined\n", new_base_link.c_str());
        return false;
    }
    
    if( task->state != State::Online )
    {
        XBot::Logger::error("Unable to change base link while performing a reach\n");
        return false;
    }
    
    task->base_frame = new_base_link;
    task->T = new_T_old * task->T;
    task->vel.setZero();
    task->acc.setZero();
    
    return true;
}


CartesianInterfaceImpl::Task::Ptr CartesianInterfaceImpl::get_task(const std::string& ee_name) const
{
    auto it = _task_map.find(ee_name);
    
    if(it == _task_map.end())
    {
        XBot::Logger::error("Task %s undefined \n", ee_name.c_str());
        return nullptr;
    }
    
    return it->second;
}

double CartesianInterfaceImpl::get_current_time() const
{
    return _current_time;
}


bool XBot::Cartesian::CartesianInterfaceImpl::abort(const std::string& end_effector)
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    if(task->state == State::Reaching)
    {
        task->state = State::Online;
    }
    
    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::getPoseReference(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref,
                          Eigen::Vector6d * base_acc_ref) const
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    base_T_ref = task->T;
    if(base_vel_ref) *base_vel_ref = task->vel;
    if(base_acc_ref) *base_acc_ref = task->acc;
    
    return true;
}

bool CartesianInterfaceImpl::getPoseTarget(const std::string& end_effector, Eigen::Affine3d& w_T_ref) const
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    if(task->state == State::Reaching)
    {
        w_T_ref = task->trajectory->getWayPoints().back().frame;
        return true;
    }
    else
    {
        XBot::Logger::warning("Task %s is NOT in REACHING mode \n", end_effector.c_str());
        return false;
    }
}


bool CartesianInterfaceImpl::setPoseReference(const std::string& end_effector, 
                                              const Eigen::Affine3d& w_T_ref, 
                                              const Eigen::Vector6d& w_vel_ref, 
                                              const Eigen::Vector6d& w_acc_ref)
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    if(task->state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in REACHING state \n");
        return false;
    }
    
    if(task->control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in NOT in position mode \n");
        return false;
    }
    
    task->T = w_T_ref;
    task->vel = w_vel_ref;
    task->acc = w_acc_ref;
    
    return true;
}

bool CartesianInterfaceImpl::setTargetPose(const std::string& end_effector, 
                                           const Eigen::Affine3d& w_T_ref, double time)
{
    auto task = get_task(end_effector);
    
    if(!task || task->state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task is in already in REACHING mode \n");
        return false;
    }
    
    if(task->control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in NOT in position mode \n");
        return false;
    }
    
    task->state = State::Reaching;
    task->trajectory->clear();
    task->trajectory->addWayPoint(get_current_time(), task->T);
    task->trajectory->addWayPoint(get_current_time() + time, w_T_ref);

    
    return true;
}

bool CartesianInterfaceImpl::setTargetPosition(const std::string& end_effector, 
                                               const Eigen::Vector3d& w_pos_ref, 
                                               double time)
{
    auto task = get_task(end_effector);
    
    if(!task || task->state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task is in already in REACHING mode \n");
        return false;
    }
    
    if(task->control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in NOT in position mode \n");
        return false;
    }

    Eigen::Affine3d w_T_ref = task->T;
    w_T_ref.translation() = w_pos_ref;
    
    task->state = State::Reaching;
    task->trajectory->clear();
    task->trajectory->addWayPoint(get_current_time(), task->T);
    task->trajectory->addWayPoint(get_current_time() + time, w_T_ref);

    
    return true;
}

bool CartesianInterfaceImpl::reset()
{
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        if(task.base_frame == "world")
        {
            _model->getPose(task.distal_frame, task.T);
        }
        else
        {
            _model->getPose(task.base_frame, task.distal_frame, task.T);
        }
        
        task.vel.setZero();
        task.acc.setZero();
        
        task.state = State::Online;
        
    }
    
    return true;
    
}

const std::map< std::string, CartesianInterfaceImpl::Task::Ptr >& CartesianInterfaceImpl::get_tasks() const
{
    return _task_map;
}

CartesianInterfaceImpl::Task::Task():
    T(Eigen::Affine3d::Identity()),
    vel(Eigen::Vector6d::Zero()),
    acc(Eigen::Vector6d::Zero()),
    trajectory(std::make_shared<Trajectory>()),
    control_type(ControlType::Position),
    state(State::Online)
{

}

CartesianInterfaceImpl::~CartesianInterfaceImpl()
{
    _logger->flush();
}


CartesianInterfaceImpl::CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, 
                                               std::vector< std::pair< std::string, std::string > > tasks):
    _model(model),
    _tasks_vector(tasks),
    _current_time(0.0),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger"))
{
    __construct_from_vectors();
}

void CartesianInterfaceImpl::__construct_from_vectors()
{
    for(auto pair : _tasks_vector)
    {
        Eigen::Affine3d T;
        if(!_model->getPose(pair.first, T))
        {
            XBot::Logger::error("CartesianInterface: unable to find frame %s inside URDF\n", pair.first.c_str());
            continue;
        }
        
        if(pair.second != "world" && !_model->getPose(pair.second, T))
        {
            XBot::Logger::error("CartesianInterface: unable to find frame %s inside URDF\n", pair.second.c_str());
            continue;
        }
        
        auto task = std::make_shared<CartesianInterfaceImpl::Task>();
        task->base_frame = pair.first;
        task->distal_frame = pair.second;
        
        _task_map[task->distal_frame] = task;
        
        _ee_list.push_back(task->distal_frame);
        
    }
    
    reset();
}


bool CartesianInterfaceImpl::update(double time, double period)
{
    /* Update reaching tasks */
    
    _current_time = time;
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        if(task.state == State::Reaching)
        {
            task.T = task.trajectory->evaluate(time, &task.vel, &task.acc);
            
            if(task.trajectory->isTrajectoryEnded(time))
            {
                task.state = State::Online;
            }
        }
        else
        {
            task.vel.setZero();
            task.acc.setZero();
        }
        
    }
    
    log_tasks();
    
    return true;
    
}

void CartesianInterfaceImpl::log_tasks()
{
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        _logger->add(task.base_frame + "_to_" + task.distal_frame + "_pos", task.T.translation());
        _logger->add(task.base_frame + "_to_" + task.distal_frame + "_or", Eigen::Quaterniond(task.T.linear()).coeffs());
        
    }
}


CartesianInterfaceImpl::CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, ProblemDescription ik_problem):
    _model(model),
    _current_time(0.0),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger"))
{
    
    for(int i = 0; i < ik_problem.getNumTasks(); i++)
    {
        for(auto task_desc : ik_problem.getTask(i))
        {
            
            switch(task_desc->type)
            {
                case TaskType::Cartesian:
                    
                    auto cart_desc = GetAsCartesian(task_desc);
                    _tasks_vector.emplace_back(cart_desc->base_link, cart_desc->distal_link);
                    break;
            }
            
            
        }
    }
    
    __construct_from_vectors();
    
}

bool CartesianInterfaceImpl::setComPositionReference(const Eigen::Vector3d& w_com_ref, 
                                                     const Eigen::Vector3d& w_vel_ref, 
                                                     const Eigen::Vector3d& w_acc_ref)
{
    throw std::runtime_error("unsupported function");
}

bool CartesianInterfaceImpl::setPositionReference(const std::string& end_effector, 
                                                  const Eigen::Vector3d& w_pos_ref,
                                                  const Eigen::Vector3d& w_vel_ref, 
                                                  const Eigen::Vector3d& w_acc_ref)
{
    throw std::runtime_error("unsupported function");
}

bool CartesianInterfaceImpl::setTargetComPosition(const Eigen::Vector3d& w_com_ref, 
                                                  double time)
{
    throw std::runtime_error("unsupported function");
}


bool CartesianInterfaceImpl::setTargetOrientation(const std::string& end_effector, 
                                                  const Eigen::Matrix3d& base_R_ref, 
                                                  double time)
{
    throw std::runtime_error("unsupported function");
}

const std::vector< std::string >& CartesianInterfaceImpl::getTaskList() const
{
    return _ee_list;
}

bool CartesianInterfaceImpl::getCurrentPose(const std::string& end_effector, Eigen::Affine3d& w_T_ee) const
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    if(task->base_frame == "world")
    {
        _model->getPose(task->distal_frame, w_T_ee);
    }
    else
    {
        _model->getPose(task->distal_frame, task->base_frame,  w_T_ee);
    }
    
    return true;
    
}

const std::string& CartesianInterfaceImpl::getBaseLink(const std::string& ee_name) const
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        throw std::invalid_argument("Undefined end effector");
    }
    
    return task->base_frame;
}

CartesianInterface::ControlType CartesianInterfaceImpl::getControlMode(const std::string& ee_name) const
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return ControlType::Disabled;
    }
    
    return task->control_type;
}


bool CartesianInterfaceImpl::setControlMode(const std::string& ee_name, CartesianInterface::ControlType ctrl_type)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return false;
    }
    
    task->control_type = ctrl_type;
    
    if(task->base_frame == "world")
    {
        _model->getPose(task->distal_frame, task->T);
    }
    else
    {
        _model->getPose(task->base_frame, task->distal_frame, task->T);
    }

    return true;
}


CartesianInterface::State CartesianInterfaceImpl::getTaskState(const std::string& end_effector) const
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return CartesianInterface::State::Online;
    }
    
    return task->state;
}


void XBot::Cartesian::CartesianInterfaceImpl::syncFrom(XBot::Cartesian::CartesianInterfaceImpl::ConstPtr other)
{
    for(const auto& pair : other->_task_map)
    {
        auto it = _task_map.find(pair.first);
        
        if(it == _task_map.end())
        {
            continue;
        }
        
        Task::Ptr this_task = it->second;
        Task::ConstPtr other_task = pair.second;
        
        if(this_task->base_frame == other_task->base_frame)
        {
            this_task->acc = other_task->acc;
            this_task->vel = other_task->vel;
            this_task->T = other_task->T;
            this_task->control_type = other_task->control_type;
            this_task->state = other_task->state;
            
            *(this_task->trajectory) = *(other_task->trajectory);
            
        }
        
    }
}

XBot::ModelInterface::Ptr CartesianInterfaceImpl::getModel() const
{
    return _model;
}



#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <boost/algorithm/string.hpp>  


using namespace XBot::Cartesian;

namespace {
    const double DEFAULT_TTL = 0.1;
}

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


CartesianInterface::ControlType CartesianInterface::ControlTypeFromString(const std::string& ctrl)
{
    
    std::string ctrl_lower = ctrl;
    boost::algorithm::to_lower(ctrl_lower);
    
    if(ctrl_lower == "disabled") return ControlType::Disabled;
    if(ctrl_lower == "position") return ControlType::Position;
    if(ctrl_lower == "velocity") return ControlType::Velocity;
    
    throw std::invalid_argument("Invalid control type '" + ctrl + "'");
    
}


CartesianInterface::State CartesianInterface::StateFromString(const std::string& state)
{
    std::string state_lower = state;
    boost::algorithm::to_lower(state_lower);
    
    if(state_lower == "online") return State::Online;
    if(state_lower == "reaching") return State::Reaching;
    
    throw std::invalid_argument("Invalid state '" + state + "'");
}

bool XBot::Cartesian::CartesianInterfaceImpl::setBaseLink(const std::string& ee_name, 
                                                          const std::string& new_base_link)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return false;
    }
    
    /* Check that new base link exists */
    Eigen::Affine3d T;
    if(!_model->getPose(new_base_link, T))
    {
        XBot::Logger::error("New base link %s in not defined\n", new_base_link.c_str());
        return false;
    }
    
    /* Check that the task is not in reaching mode */
    if( task->state != State::Online )
    {
        XBot::Logger::error("Unable to change base link while performing a reach\n");
        return false;
    }
    
    /* Recompute reference according to new base link */
    Eigen::Affine3d new_T_old;
    
    if(new_base_link == task->base_frame) // base link did not actually change
    {
        new_T_old.setIdentity();
    }
    else if(new_base_link == "world") // new base link is world
    {
        _model->getPose(task->base_frame, new_T_old); // w_T_b1 = b2_T_b1
    }
    else if(task->base_frame == "world") // old base link is world
    {
         _model->getPose(new_base_link, new_T_old); // w_T_b2 = b1_T_b2
         new_T_old = new_T_old.inverse();
    }
    else // both old and new base links are not world
    {
        _model->getPose(task->base_frame, new_base_link, new_T_old); // b2_T_b1
    }
    
    /* Update task */
    task->base_frame = new_base_link;
    task->T = new_T_old * task->T;
    task->vel.setZero();
    task->acc.setZero();
    task->new_data_available = true;
    
    return true;
}


CartesianInterfaceImpl::Task::Ptr CartesianInterfaceImpl::get_task(const std::string& ee_name) const
{
    auto it = _task_map.find(ee_name);
    
    if(ee_name == "com")
    {
        if(_com_task)
        {
            return _com_task;
        }
        else
        {
            it = _task_map.end();
        }
    }
    
    
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
        task->trajectory->clear();
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
    
    if(task->control_type == ControlType::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in DISABLED mode \n");
        return false;
    }
    
    if((task->T.translation() - w_T_ref.translation()).norm() > 0.01)
    {
        XBot::Logger::warning("Task %s: jump detected in reference \n", task->distal_frame.c_str());
    }
    
    if(task->control_type == ControlType::Position)
    {
        task->T = w_T_ref;
    }
    
    task->vel = w_vel_ref;
    task->acc = w_acc_ref;
    
    if(w_vel_ref.squaredNorm() > 0)
    {
        task->vref_time_to_live = DEFAULT_TTL;
    }
    
    task->new_data_available = true;
    
    return true;
}

bool CartesianInterfaceImpl::setWayPoints(const std::string& end_effector, 
                                          const Trajectory::WayPointVector& way_points)
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
    
    for(const auto& wp : way_points)
    {
        task->trajectory->addWayPoint(wp, get_current_time());
    }

    task->new_data_available = true;
    
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
    task->new_data_available = true;
    
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
    task->new_data_available = true;
    
    return true;
}

bool CartesianInterfaceImpl::reset()
{
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        if(task.distal_frame == "com")
        {
            continue;
        }
        
        if(task.base_frame == "world")
        {
            _model->getPose(task.distal_frame, task.T);
        }
        else
        {
            _model->getPose(task.distal_frame, task.base_frame, task.T);
        }
        
        task.vel.setZero();
        task.acc.setZero();
        
        task.state = State::Online;
        
        task.new_data_available = true;
        
    }
    
    if(_com_task)
    {
        Eigen::Vector3d com;
        _model->getCOM(com);
        _com_task->T.translation() = com;
        _com_task->vel.setZero();
        _com_task->acc.setZero();
        _com_task->state = State::Online;
        _com_task->new_data_available = true;
    }
    
    return true;
    
}

CartesianInterfaceImpl::Task::Task():
    T(Eigen::Affine3d::Identity()),
    vel(Eigen::Vector6d::Zero()),
    acc(Eigen::Vector6d::Zero()),
    trajectory(std::make_shared<Trajectory>()),
    control_type(ControlType::Position),
    state(State::Online),
    vref_time_to_live(0.0),
    new_data_available(false)
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
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger_" + std::to_string(rand())))
{
    __construct_from_vectors();
}

void CartesianInterfaceImpl::__construct_from_vectors()
{
    for(auto pair : _tasks_vector)
    {
        Eigen::Affine3d T;
        if(pair.first != "world" && !_model->getPose(pair.first, T))
        {
            XBot::Logger::error("CartesianInterface: unable to find frame %s inside URDF\n", pair.first.c_str());
            continue;
        }
        
        if(pair.second != "com" && !_model->getPose(pair.second, T))
        {
            XBot::Logger::error("CartesianInterface: unable to find frame %s inside URDF\n", pair.second.c_str());
            continue;
        }
        
        auto task = std::make_shared<CartesianInterfaceImpl::Task>();
        task->base_frame = pair.first;
        task->distal_frame = pair.second;
        
        
        if(pair.second == "com")
        {
            _com_task = task;
        }
        
        _task_map[task->distal_frame] = task;
        
        Logger::success(Logger::Severity::HIGH) <<  "Successfully added task with\n" << 
            "   BASE LINK:   " << XBot::bold_on << task->base_frame << XBot::bold_off  << "\n" << XBot::color_yellow <<
            "   DISTAL LINK: " << XBot::bold_on << task->distal_frame << XBot::bold_off << Logger::endl();
        
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
        
        task.vref_time_to_live -= period;
        
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
            if(task.vref_time_to_live <= 0.0)
            {
                task.vel.setZero();
                task.acc.setZero();
                task.vref_time_to_live = -1.0;
            }
        }
        
        task.new_data_available = false;
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
        _logger->add(task.base_frame + "_to_" + task.distal_frame + "_vel", task.vel);
        _logger->add(task.base_frame + "_to_" + task.distal_frame + "_rot", Eigen::Quaterniond(task.T.linear()).coeffs());
        _logger->add(task.base_frame + "_to_" + task.distal_frame + "_state", task.state == State::Reaching ? 1 : 0);
    }
    
    _logger->add("ci_time", _current_time);
}


CartesianInterfaceImpl::CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, ProblemDescription ik_problem):
    _model(model),
    _current_time(0.0),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger_" + std::to_string(rand()))),
    _solver_options(ik_problem.getSolverOptions())
{
    
    for(int i = 0; i < ik_problem.getNumTasks(); i++)
    {
        for(auto task_desc : ik_problem.getTask(i))
        {
            
            switch(task_desc->type)
            {
                case TaskType::Cartesian:
                {
                    auto cart_desc = GetAsCartesian(task_desc);
                    _tasks_vector.emplace_back(cart_desc->base_link, cart_desc->distal_link);
                    break;
                }    
                case TaskType::Com:
                {   
                    auto com_desc = GetAsCom(task_desc);
                    _tasks_vector.emplace_back("world", "com");
                    break;
                }   
                case TaskType::Postural:
                {   
                    if(!_model->getRobotState("home", _q_ref))
                    {
                        Logger::warning("Group state \"home\" undefined inside SRDF: setting posture reference to zero\n");
                    }
                    break;
                }   
                default:
                    Logger::warning("Unsupported task type\n");
            }
            
            
        }
    }
    
    __construct_from_vectors();
    
}

bool CartesianInterfaceImpl::setComPositionReference(const Eigen::Vector3d& w_com_ref, 
                                                     const Eigen::Vector3d& w_vel_ref, 
                                                     const Eigen::Vector3d& w_acc_ref)
{
    if(!_com_task)
    {
        return false;
    }
    
    if(_com_task->state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in REACHING state \n");
        return false;
    }
    
    if(_com_task->control_type == ControlType::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task is DISABLED \n");
        return false;
    }
    
    if(_com_task->control_type == ControlType::Position)
    {
        _com_task->T.translation() = w_com_ref;
    }
    
    _com_task->vel.head<3>() = w_vel_ref;
    _com_task->acc.head<3>() = w_acc_ref;
    _com_task->state = State::Online;
    _com_task->new_data_available = true;
    
    if(w_vel_ref.squaredNorm() > 0)
    {
        _com_task->vref_time_to_live = DEFAULT_TTL;
    }
    
    return true;
}


bool CartesianInterfaceImpl::setTargetComPosition(const Eigen::Vector3d& w_com_ref, 
                                                  double time)
{
    auto task = _com_task;
    
    if(!_com_task)
    {
        return false;
    }
    
    if(!_com_task || _com_task->state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task is in already in REACHING mode \n");
        return false;
    }
    
    if(_com_task->control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set pose reference. Task is in NOT in position mode \n");
        return false;
    }
    
    Eigen::Affine3d T = _com_task->T;
    T.translation() = w_com_ref;
    
    _com_task->state = State::Reaching;
    _com_task->trajectory->clear();
    _com_task->trajectory->addWayPoint(get_current_time(), _com_task->T);
    _com_task->trajectory->addWayPoint(get_current_time() + time, T);
    _com_task->new_data_available = true;

    
    return true;
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
        if(task->distal_frame == "com")
        {
            Eigen::Vector3d com;
            w_T_ee.setIdentity();
            _model->getCOM(com);
            w_T_ee.translation() = com;
        }
        else
        {
            _model->getPose(task->distal_frame, w_T_ee);
        }
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
        XBot::Logger::error("Undefined end effector %s\n", ee_name.c_str());
        return false;
    }
    
    task->control_type = ctrl_type;
    
    if(ee_name == "com")
    {
        Eigen::Vector3d com;
        _model->getCOM(com);
        task->T.translation() = com;
    }
    else if(task->base_frame == "world")
    {
        _model->getPose(task->distal_frame, task->T);
    }
    else
    {
        _model->getPose(task->base_frame, task->distal_frame, task->T);
    }
    
    task->new_data_available = true;

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
        
        if(!other_task->new_data_available)
        {
            continue;
        }
        
        
        if(this_task->base_frame != other_task->base_frame)
        {
            if(!setBaseLink(other_task->distal_frame, other_task->base_frame))
            {
                Logger::error("Unable to change base link for task %s (%s -> %s)\n", 
                              this_task->distal_frame.c_str(),
                              this_task->base_frame.c_str(),
                              other_task->base_frame.c_str()
                );
                
                continue;
            }
            
        }
        
        if(this_task->control_type != other_task->control_type)
        {
            if(!setControlMode(other_task->distal_frame, other_task->control_type))
            {
                Logger::error("Unable to change control mode for task %s (%s -> %s)\n", 
                              this_task->distal_frame.c_str(),
                              ControlTypeAsString(this_task->control_type).c_str(),
                              ControlTypeAsString(other_task->control_type).c_str()
                );
                
                continue;
            }
            
        }
        
        this_task->acc = other_task->acc;
        this_task->vel = other_task->vel;
        this_task->T = other_task->T;
        this_task->state = other_task->state;
        this_task->vref_time_to_live = other_task->vref_time_to_live;
        
        *(this_task->trajectory) = *(other_task->trajectory);
        
    }
}

XBot::ModelInterface::Ptr CartesianInterfaceImpl::getModel() const
{
    return _model;
}

bool CartesianInterfaceImpl::getComPositionReference(Eigen::Vector3d& w_com_ref, 
                                                     Eigen::Vector3d* base_vel_ref, 
                                                     Eigen::Vector3d* base_acc_ref) const
{
    if(!_com_task)
    {
        Logger::error("Undefined task com\n");
        return false;
    }
    
    w_com_ref = _com_task->T.translation();
    if(base_vel_ref) *base_vel_ref = _com_task->vel.head<3>();
    if(base_acc_ref) *base_acc_ref = _com_task->acc.head<3>();
    
    return true;
}

bool CartesianInterfaceImpl::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
{
    if(!_com_task)
    {
        return false;
    }
    
    if(_com_task->state == State::Reaching)
    {
        w_com_ref = _com_task->trajectory->getWayPoints().back().frame.translation();
        return true;
    }
    else
    {
        XBot::Logger::warning("Task com is NOT in REACHING mode \n");
        return false;
    }
}

const YAML::Node& XBot::Cartesian::CartesianInterfaceImpl::get_config() const
{
    return _solver_options;
}


bool XBot::Cartesian::CartesianInterfaceImpl::has_config() const
{
    return _solver_options;
}

bool XBot::Cartesian::CartesianInterfaceImpl::postural_task_defined() const
{
    return _q_ref.size() != 0;
}


bool XBot::Cartesian::CartesianInterfaceImpl::getReferencePosture(Eigen::VectorXd& qref) const
{
    if(!postural_task_defined())
    {
        return false;
    }
    
    qref = _q_ref;
    
    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::getReferencePosture(XBot::JointNameMap& qref) const
{
    if(!postural_task_defined())
    {
        return false;
    }
    
    return _model->eigenToMap(_q_ref, qref);;
}

bool XBot::Cartesian::CartesianInterfaceImpl::setReferencePosture(const XBot::JointNameMap& qref)
{
    if(!postural_task_defined())
    {
        return false;
    }
    
    _model->mapToEigen(qref, _q_ref);
    return true;
}



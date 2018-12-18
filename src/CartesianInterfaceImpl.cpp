#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Cartesian.h>
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
            throw std::runtime_error("Invalid control type");
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
            throw std::runtime_error("Invalid state");
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
    if(ee_name == "com")
    {
        Logger::error("Base link for task Com cannot be changed\n");
        return false;
    }
    
    auto task = get_task(ee_name);
    
    if(!task || !task->change_base_link(new_base_link, _model))
    {
        return false;
    }
    
    
    Logger::success(Logger::Severity::HIGH, "Base link changed to %s for task %s\n", 
                    new_base_link.c_str(), 
                    task->get_name().c_str());
    
    
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
    
    task->abort();
    
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
    
    base_T_ref = task->get_pose_otg();
    if(base_vel_ref) *base_vel_ref = task->get_velocity();
    if(base_acc_ref) *base_acc_ref = task->get_acceleration();
    
    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::getPoseReferenceRaw(const std::string& end_effector, 
                          Eigen::Affine3d& base_T_ref, 
                          Eigen::Vector6d * base_vel_ref,
                          Eigen::Vector6d * base_acc_ref) const
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    base_T_ref = task->get_pose();
    if(base_vel_ref) *base_vel_ref = task->get_velocity();
    if(base_acc_ref) *base_acc_ref = task->get_acceleration();
    
    return true;
}

bool CartesianInterfaceImpl::getPoseTarget(const std::string& end_effector, Eigen::Affine3d& w_T_ref) const
{
    auto task = get_task(end_effector);
    
    if(!task || !task->get_pose_target(w_T_ref))
    {
        return false;
    }
    
    return true;
    
}


bool CartesianInterfaceImpl::setPoseReference(const std::string& end_effector, 
                                              const Eigen::Affine3d& w_T_ref, 
                                              const Eigen::Vector6d& w_vel_ref, 
                                              const Eigen::Vector6d& w_acc_ref)
{
    auto task = get_task(end_effector);
    
    if(!task || !task->set_reference(w_T_ref, w_vel_ref, w_acc_ref))
    {
        return false;
    }
    
    return true;
}

bool CartesianInterfaceImpl::setPoseReferenceRaw(const std::string& end_effector, 
                                              const Eigen::Affine3d& w_T_ref, 
                                              const Eigen::Vector6d& w_vel_ref, 
                                              const Eigen::Vector6d& w_acc_ref)
{
    auto task = get_task(end_effector);
    
    if(!task || !task->set_reference(w_T_ref, w_vel_ref, w_acc_ref))
    {
        return false;
    }
    
    task->reset_otg();
    
    return true;
}


bool CartesianInterfaceImpl::setWayPoints(const std::string& end_effector, 
                                          const Trajectory::WayPointVector& way_points)
{
    auto task = get_task(end_effector);
    
    if(!task || !task->set_waypoints(get_current_time(), way_points))
    {
        return false;
    }
    
    return true;
}


bool CartesianInterfaceImpl::setTargetPose(const std::string& end_effector, 
                                           const Eigen::Affine3d& w_T_ref, double time)
{
    auto task = get_task(end_effector);
    
    if(!task || !task->set_target_pose(get_current_time(), get_current_time() + time, w_T_ref))
    {
        return false;
    }
    
    return true;
}

bool CartesianInterfaceImpl::setTargetPosition(const std::string& end_effector, 
                                               const Eigen::Vector3d& w_pos_ref, 
                                               double time)
{
    auto task = get_task(end_effector);
    
    if(!task)
    {
        return false;
    }
   
    Eigen::Affine3d w_T_ref = task->get_pose();
    w_T_ref.translation() = w_pos_ref;
    
   return setTargetPose(end_effector, w_T_ref, time);
}

bool CartesianInterfaceImpl::reset(double time)
{
    _current_time = time;
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        task.reset(_model);
    }
    
    if(_com_task)
    {
        _com_task->reset(_model);
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
    new_data_available(false),
    __otg_ref(EigenVector7d::Zero()),
    __otg_des(EigenVector7d::Zero()),
    __otg_maxvel(EigenVector7d::Constant(1.0)),
    __otg_maxacc(EigenVector7d::Constant(5.0))
{
    
}

void XBot::Cartesian::CartesianInterfaceImpl::Task::set_otg_dt(double expected_dt)
{
    otg = std::make_shared<Reflexxes::Utils::TrajectoryGenerator>(7, expected_dt, EigenVector7d::Zero());
    reset_otg();
    otg->setVelocityLimits(__otg_maxvel);
    otg->setAccelerationLimits(__otg_maxacc);
}

void XBot::Cartesian::CartesianInterfaceImpl::Task::set_otg_acc_limits(double linear, double angular)
{
    __otg_maxacc << linear, linear, linear, angular/2.0, angular/2.0, angular/2.0, angular/2.0;
    
    if(otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_acc_limits (%f, %f): success\n", linear, angular);
        otg->setAccelerationLimits(__otg_maxacc);
    }
}

void XBot::Cartesian::CartesianInterfaceImpl::Task::set_otg_vel_limits(double linear, double angular)
{
    __otg_maxvel << linear, linear, linear, angular/2.0, angular/2.0, angular/2.0, angular/2.0;
    
    if(otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_vel_limits (%f, %f): success\n", linear, angular);
        otg->setVelocityLimits(__otg_maxvel);
    }
}



CartesianInterfaceImpl::~CartesianInterfaceImpl()
{
    _logger->flush();
    Logger::success(Logger::Severity::HIGH, "Cleanly exiting from CartesI/O\n");
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
    /* Delete possible duplicates (e.g. different subtasks on different priorities */
    std::sort(_tasks_vector.begin(), _tasks_vector.end());
    _tasks_vector.erase(std::unique(_tasks_vector.begin(), _tasks_vector.end()), _tasks_vector.end());
    
    /* Consistency check: different base links for same distal NOT allowed */
    std::map<std::string, int> counts;
    for(auto tpair : _tasks_vector)
    {
        auto it = counts.find(tpair.second);
        if(it == counts.end())
        {
            counts[tpair.second] = 0;
        }
        else
        {
            throw std::runtime_error("different base links for same distal not allowed (" + tpair.second + ")");
        }

    }


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
        
        auto task = std::make_shared<CartesianInterfaceImpl::Task>(pair.first, pair.second);
        
        if(pair.second == "com")
        {
            _com_task = task;
        }
        
        _task_map[task->get_distal()] = task;
        
        Logger::success(Logger::Severity::HIGH) <<  "Successfully added task with\n" << 
            "   BASE LINK:   " << XBot::bold_on << task->get_base() << XBot::bold_off  << "\n" << XBot::color_yellow <<
            "   DISTAL LINK: " << XBot::bold_on << task->get_distal() << XBot::bold_off << Logger::endl();
        
        _ee_list.push_back(task->get_distal());
        
    }
    
    reset(0.0);
    init_log_tasks();
}


bool CartesianInterfaceImpl::update(double time, double period)
{
    /* Update reaching tasks */
    
    _current_time = time;
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        task.update(time, period);
    }
    
    log_tasks();
    
    return true;
    
}

void CartesianInterfaceImpl::log_tasks()
{
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        _logger->add(task.get_distal() + "_pos", task.get_pose().translation());
        _logger->add(task.get_distal() + "_pos_otg", task.get_pose_otg().translation());
        _logger->add(task.get_distal() + "_vel", task.get_velocity());
        _logger->add(task.get_distal() + "_rot", Eigen::Quaterniond(task.get_pose().linear()).coeffs());
        _logger->add(task.get_distal() + "_rot_otg", Eigen::Quaterniond(task.get_pose_otg().linear()).coeffs());
        _logger->add(task.get_distal() + "_state", task.get_state() == State::Reaching ? 1 : 0);
        
    }
    
    _logger->add("ci_time", _current_time);
}

void XBot::Cartesian::CartesianInterfaceImpl::init_log_tasks()
{
    const int BUF_SIZE = 2e5;
    
    for(auto& pair : _task_map)
    {
        CartesianInterfaceImpl::Task& task = *(pair.second);
        
        _logger->createVectorVariable(task.get_distal() + "_pos", 3, 1, BUF_SIZE);
        _logger->createVectorVariable(task.get_distal() + "_pos_otg", 3, 1, BUF_SIZE);
        _logger->createVectorVariable(task.get_distal() + "_vel", 6, 1, BUF_SIZE);
        _logger->createVectorVariable(task.get_distal() + "_rot", 4, 1, BUF_SIZE);
        _logger->createVectorVariable(task.get_distal() + "_rot_otg", 4, 1, BUF_SIZE);
        _logger->createScalarVariable(task.get_distal() + "_state", 1, BUF_SIZE);
    }
    
    _logger->createScalarVariable("ci_time", 1, BUF_SIZE);
}



CartesianInterfaceImpl::CartesianInterfaceImpl(XBot::ModelInterface::Ptr model, ProblemDescription ik_problem):
    _model(model),
    _current_time(0.0),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger_" + std::to_string(rand()))),
    _solver_options(ik_problem.getSolverOptions())
{
    /* Parse tasks */
    for(int i = 0; i < ik_problem.getNumTasks(); i++)
    {
        for(auto task_desc : ik_problem.getTask(i))
        {
            
            add_task(task_desc);
            
        }
    }
    
    /* Check if there are constraints which are actually tasks */
    for(int i = 0; i < ik_problem.getBounds().size(); i++)
    {
        if(ik_problem.getBounds().at(i)->type == "ConstraintFromTask")
        {
            auto task = GetTaskFromConstraint(ik_problem.getBounds().at(i));
            if(task)
            {
                add_task(task);
            }
            else
            {
                Logger::error("Unable to get task from constraint #%d\n", i);
            }
               
        }
    }
    
    __construct_from_vectors();
    
}

void XBot::Cartesian::CartesianInterfaceImpl::add_task(TaskDescription::Ptr task_desc)
{
    switch(task_desc->interface)
    {
        case TaskInterface::Cartesian:
        {
            auto cart_desc = GetAsCartesian(task_desc);
            _tasks_vector.emplace_back(cart_desc->base_link, cart_desc->distal_link);
            break;
        }
        case TaskInterface::Postural:
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


bool CartesianInterfaceImpl::setComPositionReference(const Eigen::Vector3d& w_com_ref, 
                                                     const Eigen::Vector3d& w_vel_ref, 
                                                     const Eigen::Vector3d& w_acc_ref)
{
    if(!_com_task)
    {
        return false;
    }
    
    Eigen::Affine3d Tref;
    Eigen::Vector6d velref, accref;
    velref.setZero();
    accref.setZero();
    velref.head<3>() = w_vel_ref;
    accref.head<3>() = w_acc_ref;
    
    Tref.setIdentity();
    Tref.translation() = w_com_ref;
    
    return _com_task->set_reference(Tref, velref, accref);
}


bool CartesianInterfaceImpl::setTargetComPosition(const Eigen::Vector3d& w_com_ref, 
                                                  double time)
{
    
    if(!_com_task)
    {
        return false;
    }
    
    Eigen::Affine3d T;
    T.setIdentity();
    T.translation() = w_com_ref;
    
    
    return _com_task->set_target_pose(get_current_time(), get_current_time() + time, T);
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
    
    if(task->get_base() == "world")
    {
        if(task->get_distal() == "com")
        {
            Eigen::Vector3d com;
            w_T_ee.setIdentity();
            _model->getCOM(com);
            w_T_ee.translation() = com;
        }
        else
        {
            _model->getPose(task->get_distal(), w_T_ee);
        }
    }
    else
    {
        _model->getPose(task->get_distal(), task->get_base(),  w_T_ee);
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
    
    return task->get_base();
}

CartesianInterface::ControlType CartesianInterfaceImpl::getControlMode(const std::string& ee_name) const
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return ControlType::Disabled;
    }
    
    return task->get_ctrl();
}


bool CartesianInterfaceImpl::setControlMode(const std::string& ee_name, CartesianInterface::ControlType ctrl_type)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return false;
    }
    
    task->set_ctrl(ctrl_type, _model);
    
    Logger::success(Logger::Severity::HIGH, 
                    "Control mode changed to %s for task %s\n", 
                    ControlTypeAsString(ctrl_type).c_str(), task->get_distal().c_str());

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
    
    return task->get_state();
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
        
        if(!other_task->is_new_data_available())
        {
            continue;
        }
        
        Logger::info(Logger::Severity::DEBUG, "New data available for task %s\n", other_task->get_distal().c_str());
        
        
        if(this_task->get_base() != other_task->get_base())
        {
            if(!setBaseLink(other_task->get_distal(), other_task->get_base()))
            {
                Logger::error("Unable to change base link for task %s (%s -> %s)\n", 
                              this_task->get_distal().c_str(),
                              this_task->get_base().c_str(),
                              other_task->get_base().c_str()
                );
                
                continue;
            }
            
        }
        
        if(this_task->get_ctrl() != other_task->get_ctrl())
        {
            if(!setControlMode(other_task->get_distal(), other_task->get_ctrl()))
            {
                Logger::error("Unable to change control mode for task %s (%s -> %s)\n", 
                              this_task->get_distal().c_str(),
                              ControlTypeAsString(this_task->get_ctrl()).c_str(),
                              ControlTypeAsString(other_task->get_ctrl()).c_str()
                );
                
                continue;
            }
            
        }
        
        this_task->sync_from(*other_task);
        
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
    
    w_com_ref = _com_task->get_pose().translation();
    if(base_vel_ref) *base_vel_ref = _com_task->get_velocity().head<3>();
    if(base_acc_ref) *base_acc_ref = _com_task->get_acceleration().head<3>();
    
    return true;
}

bool CartesianInterfaceImpl::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
{
    Eigen::Affine3d Tgoal;
    
    if(!_com_task || !_com_task->get_pose_target(Tgoal))
    {
        return false;
    }
    
    w_com_ref = Tgoal.translation();
    return true;
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

bool XBot::Cartesian::CartesianInterfaceImpl::Task::change_base_link(const std::string& new_base_link, 
                                                                     XBot::ModelInterface::ConstPtr model)
{   
    /* Check that new base link exists */
    Eigen::Affine3d T;
    if(!model->getPose(new_base_link, T))
    {
        XBot::Logger::error("New base link %s in not defined\n", new_base_link.c_str());
        return false;
    }
    
    /* Check that the task is not in reaching mode */
    if( state != State::Online )
    {
        XBot::Logger::error("Task %s: unable to change base link while performing a reach\n", distal_frame.c_str());
        return false;
    }
    
    /* Update task */
    base_frame = new_base_link;
    vel.setZero();
    acc.setZero();
    new_data_available = true;
    
    /* Reset reference */
    reset(model);
    
    return true;
}


void CartesianInterfaceImpl::Task::abort()
{
    if(state == State::Reaching)
    {
        state = State::Online;
        trajectory->clear();
    }
}

bool CartesianInterfaceImpl::Task::get_pose_target(Eigen::Affine3d& pose_target) const
{
    if(state == State::Reaching)
    {
        pose_target = trajectory->getWayPoints().back().frame;
        return true;
    }
    else
    {
        XBot::Logger::warning("Task %s is NOT in REACHING mode \n", distal_frame.c_str());
        return false;
    }
}

bool CartesianInterfaceImpl::Task::set_reference(const Eigen::Affine3d& pose, 
                                                 const Eigen::Vector6d& vel_ref, 
                                                 const Eigen::Vector6d& acc_ref)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task %s is in REACHING state \n", distal_frame.c_str());
        return false;
    }
    
    if(control_type == ControlType::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task %s is in DISABLED mode \n", distal_frame.c_str());
        return false;
    }
    
    if(control_type == ControlType::Position)
    {
        T = pose;
    }
    else
    {
        Logger::warning(Logger::Severity::DEBUG, "Task %s: not setting position reference\n", distal_frame.c_str());
    }
    
    vel = vel_ref;
    acc = acc_ref;
    
    if(!vel_ref.isZero())
    {
        vref_time_to_live = DEFAULT_TTL;
    }
    
    new_data_available = true;
    
    return true;
}


bool CartesianInterfaceImpl::Task::set_waypoints(double time, const Trajectory::WayPointVector& wps)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task %s is in already in REACHING mode \n", distal_frame.c_str());
        return false;
    }
    
    if(control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task %s is in NOT in position mode \n", distal_frame.c_str());
        return false;
    }
    
    state = State::Reaching;
    trajectory->clear();
    trajectory->addWayPoint(time, T);
    
    for(const auto& wp : wps)
    {
        trajectory->addWayPoint(wp, time);
    }

    trajectory->compute();
    new_data_available = true;
    return true;
}


bool CartesianInterfaceImpl::Task::set_target_pose(double current_time, double target_time, const Eigen::Affine3d& pose)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task %s is in already in REACHING mode \n", distal_frame.c_str());
        return false;
    }
    
    if(control_type != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task %s is in NOT in position mode \n", distal_frame.c_str());
        return false;
    }
    
    state = State::Reaching;
    trajectory->clear();
    trajectory->addWayPoint(current_time, T);
    trajectory->addWayPoint(target_time, pose);
    trajectory->compute();
    new_data_available = true;
    
    return true;
}

void CartesianInterfaceImpl::Task::reset(XBot::ModelInterface::ConstPtr model)
{
    if(distal_frame == "com")
    {
        Eigen::Vector3d com;
        model->getCOM(com);
        T.translation() = com;
        vel.setZero();
        acc.setZero();
        state = State::Online;
        new_data_available = true;
        
        reset_otg();
        
        return;
    }
    
    
    if(base_frame == "world")
    {
        model->getPose(distal_frame, T);
    }
    else
    {
        model->getPose(distal_frame, base_frame, T);
    }
    
    vel.setZero();
    acc.setZero();
    
    state = State::Online;
    
    new_data_available = true;
    
    reset_otg();
}

CartesianInterfaceImpl::Task::Task(const std::string& base, const std::string& distal):
    Task()
{
    base_frame = base;
    distal_frame = distal;
}

void CartesianInterfaceImpl::Task::update(double time, double period)
{
    vref_time_to_live -= period;
        
    if(state == State::Reaching)
    {
        T = trajectory->evaluate(time, &vel, &acc);
        
        if(trajectory->isTrajectoryEnded(time) && check_reach())
        {
            state = State::Online;
        }
    }
    else
    {
        if(vref_time_to_live <= 0.0)
        {
            vel.setZero();
            acc.setZero();
            vref_time_to_live = -1.0;
        }
    }
    
    apply_otg();
    
    new_data_available = false;
}

bool CartesianInterfaceImpl::Task::check_reach() const
{
    return trajectory->getWayPoints().back().frame.isApprox(get_pose_otg());
}

CartesianInterface::ControlType CartesianInterfaceImpl::Task::get_ctrl() const
{
    return control_type;
}

void CartesianInterfaceImpl::Task::set_ctrl(ControlType ctrl, XBot::ModelInterface::ConstPtr model)
{
    control_type = ctrl;
    
    reset(model);
}

void CartesianInterfaceImpl::Task::sync_from(const CartesianInterfaceImpl::Task& other)
{
    acc = other.acc;
    vel = other.vel;
    T = other.T;
    state = other.state;
    vref_time_to_live = other.vref_time_to_live;
    
    *(trajectory) = *(other.trajectory); // TBD better implementation
}

bool CartesianInterfaceImpl::Task::is_new_data_available() const
{
    return new_data_available;
}

const Eigen::Vector6d& CartesianInterfaceImpl::Task::get_acceleration() const
{
    return acc;
}

const std::string& CartesianInterfaceImpl::Task::get_base() const
{
    return base_frame;
}

const std::string& CartesianInterfaceImpl::Task::get_distal() const
{
    return distal_frame;
}

const std::string& CartesianInterfaceImpl::Task::get_name() const
{
    return distal_frame;
}

const Eigen::Affine3d& CartesianInterfaceImpl::Task::get_pose() const
{
    return T;
}

CartesianInterface::State CartesianInterfaceImpl::Task::get_state() const
{
    return state;
}

const Eigen::Vector6d& CartesianInterfaceImpl::Task::get_velocity() const
{
    return vel;
}

void XBot::Cartesian::CartesianInterfaceImpl::Task::apply_otg()
{
    
    if(!otg) return;
    
    // TBD: also velocity level
    Eigen::Quaterniond q_actual(__otg_des.tail<4>());
    Eigen::Quaterniond q_des(T.linear());
    Eigen::Vector4d q_des_coeffs = q_des.coeffs();
    if(q_actual.dot(q_des) < 0)
    {
        q_des_coeffs = -q_des_coeffs;
    }
    
    __otg_des << T.translation(), q_des_coeffs;

    otg->setReference(__otg_des, EigenVector7d::Zero());
    otg->update(__otg_ref, __otg_vref);
    
    if(__otg_ref.tail<4>().norm() < 0.01)
    {
        Logger::warning("Experimental orientation otg: norm < 0.01 for task %s\n", get_distal().c_str());
    }

    
}

void CartesianInterfaceImpl::Task::reset_otg()
{
    
    if(!otg) return;
    
    __otg_des << T.translation(), Eigen::Quaterniond(T.linear()).coeffs();
    __otg_ref = __otg_des;
    

    otg->reset(__otg_des);

}

const Eigen::Affine3d CartesianInterfaceImpl::Task::get_pose_otg() const
{
    
    if(!otg)
    {
        return get_pose();
    }
    
    Eigen::Affine3d Totg;
    Eigen::Quaterniond q(__otg_ref.tail<4>());
    q.normalize();
    
    Totg.setIdentity();
    
    Totg.translation() = __otg_ref.head<3>();
    Totg.linear() = q.toRotationMatrix();
    
    return Totg;
    
}

void XBot::Cartesian::CartesianInterfaceImpl::enableOtg(double expected_dt)
{
    XBot::Logger::info(Logger::Severity::LOW, "Online trajectory generator enabled\n");
    
    for(auto tpair : _task_map)
    {
        tpair.second->set_otg_dt(expected_dt);
    }
}


void XBot::Cartesian::CartesianInterfaceImpl::setAccelerationLimits(const std::string& ee_name, 
                                                                    double max_acc_lin, 
                                                                    double max_acc_ang)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return;
    }
    
    XBot::Logger::info(Logger::Severity::LOW, "Setting acceleration limits for task %s (lin: %f, ang: %f)\n", 
        ee_name.c_str(),
        max_acc_lin,
        max_acc_ang
    );
    
    task->set_otg_acc_limits(max_acc_lin, max_acc_ang);
}


void XBot::Cartesian::CartesianInterfaceImpl::setVelocityLimits(const std::string& ee_name, 
                                                                double max_vel_lin, 
                                                                double max_vel_ang)
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return;
    }
    
    XBot::Logger::info(Logger::Severity::LOW, "Setting velocity limits for task %s (lin: %f, ang: %f)\n", 
        ee_name.c_str(),
        max_vel_lin,
        max_vel_ang
    );
    
    task->set_otg_vel_limits(max_vel_lin, max_vel_ang);
}   


void XBot::Cartesian::CartesianInterfaceImpl::Task::get_otg_acc_limits(double& linear, double& angular) const
{
    linear = __otg_maxacc[0];
    angular = __otg_maxacc[3]*2;
}

void XBot::Cartesian::CartesianInterfaceImpl::Task::get_otg_vel_limits(double& linear, double& angular) const
{
    linear = __otg_maxvel[0];
    angular = __otg_maxvel[3]*2;
}

void XBot::Cartesian::CartesianInterfaceImpl::getAccelerationLimits(const std::string& ee_name, 
                                                                    double& max_acc_lin, 
                                                                    double& max_acc_ang) const
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return;
    }
    
    task->get_otg_acc_limits(max_acc_lin, max_acc_ang);
}

void XBot::Cartesian::CartesianInterfaceImpl::getVelocityLimits(const std::string& ee_name, 
                                                                double& max_vel_lin, 
                                                                double& max_vel_ang) const
{
    auto task = get_task(ee_name);
    
    if(!task)
    {
        return;
    }
    
    task->get_otg_vel_limits(max_vel_lin, max_vel_ang);
}

bool XBot::Cartesian::CartesianInterfaceImpl::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    Eigen::Affine3d w_T_fb;
    if(!_model->getFloatingBasePose(w_T_fb))
    {
        return false;
    }
    
    if(!_model->setFloatingBaseState(w_T_new_world.inverse() * w_T_fb, Eigen::Vector6d::Zero()))
    {
        return false;
    }
    
    return reset(get_current_time());
}



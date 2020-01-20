#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/problem/Interaction.h>
#include <boost/algorithm/string.hpp>  

#include <cartesian_interface/Context.h>
#include <cartesian_interface/Enum.h>

using namespace XBot::Cartesian;

namespace {
const double DEFAULT_TTL = 0.1;
}

CartesianInterfaceImpl::CartesianInterfaceImpl(XBot::ModelInterface::Ptr model,
                                               ProblemDescription ik_problem):
    _model(model),
    _current_time(0.0),
    _logger(XBot::MatLogger::getLogger("/tmp/xbot_cartesian_logger_" + std::to_string(rand()))),
    _solver_options(ik_problem.getSolverOptions()),
    _ik_problem(ik_problem)
{
    if(!ik_problem.validate())
    {
        throw std::runtime_error("Invalid ik probelem");
    }

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
        auto& constr = ik_problem.getBounds().at(i);

        if(auto constr_from_task = std::dynamic_pointer_cast<ConstraintFromTask>(constr))
        {
            auto task = GetTaskFromConstraint(constr_from_task);
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

    reset(0.0);

    init_log_tasks();

}

void XBot::Cartesian::CartesianInterfaceImpl::add_task(TaskDescription::Ptr task_desc)
{

    _task_map[task_desc->getName()] = task_desc;
    _task_list.push_back(task_desc->getName());

    if(auto cart_desc = std::dynamic_pointer_cast<CartesianTask>(task_desc))
    {
        _cart_task_map[cart_desc->getName()] = cart_desc;

        auto com_desc = std::dynamic_pointer_cast<ComTask>(task_desc);

        if(com_desc)
        {
            _com_task_map[cart_desc->getName()] = com_desc;
        }

        Logger::success(Logger::Severity::HIGH) <<  "Successfully added task with\n" <<
                                                    "   BASE LINK:   " << XBot::bold_on << cart_desc->getBaseLink() << XBot::bold_off  << "\n" << XBot::color_yellow <<
                                                    "   DISTAL LINK: " << XBot::bold_on << cart_desc->getDistalLink() << XBot::bold_off << Logger::endl();
    }
    else if(auto postural_desc = std::dynamic_pointer_cast<PosturalTask>(task_desc))
    {
        _postural_task_map[task_desc->getName()] = postural_desc;
    }
    else
    {
        Logger::warning("Unsupported task type for task '%s'\n", task_desc->getType().c_str());
    }


}

std::string CartesianInterface::ControlTypeAsString(ControlType ctrl)
{
    switch(ctrl)
    {
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


std::string CartesianInterface::StateAsString(State ctrl)
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

ControlType CartesianInterface::ControlTypeFromString(const std::string& ctrl)
{
    
    std::string ctrl_lower = ctrl;
    boost::algorithm::to_lower(ctrl_lower);
    
    if(ctrl_lower == "position") return ControlType::Position;
    if(ctrl_lower == "velocity") return ControlType::Velocity;
    
    throw std::invalid_argument("Invalid control type '" + ctrl + "'");
    
}


State CartesianInterface::StateFromString(const std::string& state)
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
    
    auto task = get_cart_task(ee_name);
    
    if(!task || !task->setBaseLink(new_base_link))
    {
        return false;
    }
    
    
    Logger::success(Logger::Severity::HIGH, "Base link changed to '%s' for task '%s' \n",
                    new_base_link.c_str(),
                    task->getName().c_str());
    
    
    return true;
}

ActivationState CartesianInterfaceImpl::getActivationState(const std::string & ee_name) const
{
    auto task = get_task(ee_name);

    if(!task)
    {
        return ActivationState::Disabled;
    }

    return task->getActivationState();

}

bool CartesianInterfaceImpl::setActivationState(const std::string & ee_name, ActivationState activ_state)
{
    auto task = get_task(ee_name);

    if(!task)
    {
        return false;
    }

    return task->setActivationState(activ_state);
}


CartesianTask::Ptr CartesianInterfaceImpl::get_cart_task(const std::string& ee_name) const
{
    if(ee_name == "com" || ee_name == "Com")
    {
        return get_com_task();
    }

    auto it = _cart_task_map.find(ee_name);
    
    if(it == _cart_task_map.end())
    {
        XBot::Logger::error("Cartesian task '%s' undefined \n", ee_name.c_str());
        return nullptr;
    }
    
    return it->second;
}

ComTask::Ptr CartesianInterfaceImpl::get_com_task() const
{
    if(_com_task_map.empty())
    {
        Logger::error("Undefined task com\n");
        return ComTask::Ptr();
    }

    if(_com_task_map.size() > 1)
    {
        Logger::warning("Many com tasks defined, results might be inaccurate! \n");
    }

    auto com_task = _com_task_map.begin()->second;

    return com_task;
}

TaskDescription::Ptr CartesianInterfaceImpl::get_task(const std::string& ee_name) const
{
    auto it = _task_map.find(ee_name);

    if(it == _task_map.end())
    {
        XBot::Logger::error("Task '%s' undefined \n", ee_name.c_str());
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
    auto task = get_cart_task(end_effector);
    
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
    auto task = get_cart_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    return task->getPoseReference(base_T_ref, base_vel_ref, base_acc_ref);
}

bool XBot::Cartesian::CartesianInterfaceImpl::getPoseReferenceRaw(const std::string& end_effector,
                                                                  Eigen::Affine3d& base_T_ref,
                                                                  Eigen::Vector6d * base_vel_ref,
                                                                  Eigen::Vector6d * base_acc_ref) const
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->getPoseReferenceRaw(base_T_ref, base_vel_ref, base_acc_ref);
}

bool CartesianInterfaceImpl::getPoseTarget(const std::string& end_effector, Eigen::Affine3d& w_T_ref) const
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->getPoseTarget(w_T_ref);
    
}

int CartesianInterfaceImpl::getCurrentSegmentId(const std::string& end_effector) const
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->getCurrentSegmentId();
}


bool CartesianInterfaceImpl::setPoseReference(const std::string& end_effector,
                                              const Eigen::Affine3d& w_T_ref)
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->setPoseReference(w_T_ref);
}

bool XBot::Cartesian::CartesianInterfaceImpl::setVelocityReference(const std::string& end_effector,
                                                                   const Eigen::Vector6d& base_vel_ref)
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->setVelocityReference(base_vel_ref);
}



bool CartesianInterfaceImpl::setPoseReferenceRaw(const std::string& end_effector,
                                                 const Eigen::Affine3d& w_T_ref)
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->setPoseReferenceRaw(w_T_ref);
}


bool CartesianInterfaceImpl::setWayPoints(const std::string& end_effector,
                                          const Trajectory::WayPointVector& way_points)
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->setWayPoints(way_points);
}


bool CartesianInterfaceImpl::setTargetPose(const std::string& end_effector,
                                           const Eigen::Affine3d& w_T_ref, double time)
{
    auto task = get_cart_task(end_effector);

    if(!task)
    {
        return false;
    }

    return task->setPoseTarget(w_T_ref, time);
}

bool CartesianInterfaceImpl::reset(double time)
{
    _current_time = time;
    
    for(auto& pair : _task_map)
    {
        auto& task = *(pair.second);
        
        task.reset();
    }

    return true;
    
}

bool CartesianInterfaceImpl::update(double time, double period)
{
    _current_time = time;
    
    for(auto& pair : _task_map)
    {
        auto& task = *(pair.second);
        
        task.update(time, period);
    }
    
    log_tasks();
    
    return true;
    
}

void CartesianInterfaceImpl::log_tasks()
{
    for(auto& pair : _task_map)
    {
        auto& task = *(pair.second);
        
        task.log(_logger);
    }
    
    _logger->add("ci_time", _current_time);
}

void XBot::Cartesian::CartesianInterfaceImpl::init_log_tasks()
{
    const int BUF_SIZE = 2e5;
    
    for(auto& pair : _task_map)
    {
        auto& task = *(pair.second);
        
        task.log(_logger, true, BUF_SIZE);
    }
    
    _logger->createScalarVariable("ci_time", 1, BUF_SIZE);
}

bool CartesianInterfaceImpl::setComPositionReference(const Eigen::Vector3d& w_com_ref)
{
    if(_com_task_map.empty())
    {
        return false;
    }
    
    Eigen::Affine3d Tref;
    Tref.setIdentity();
    Tref.translation() = w_com_ref;

    for(auto pair : _com_task_map)
    {
        pair.second->setPoseReference(Tref);
    }

    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::setComVelocityReference(const Eigen::Vector3d& base_vel_ref)
{
    if(_com_task_map.empty())
    {
        return false;
    }

    Eigen::Vector6d twist;
    twist << base_vel_ref, 0., 0., 0.;
    
    for(auto pair : _com_task_map)
    {
        pair.second->setVelocityReference(twist);
    }

    return true;
}

bool CartesianInterfaceImpl::setTargetComPosition(const Eigen::Vector3d& w_com_ref,
                                                  double time)
{
    
    if(_com_task_map.empty())
    {
        return false;
    }
    
    Eigen::Affine3d T;
    T.setIdentity();
    T.translation() = w_com_ref;
    
    for(auto pair : _com_task_map)
    {
        pair.second->setPoseTarget(T, time);
    }

    return true;
}

const std::vector< std::string >& CartesianInterfaceImpl::getTaskList() const
{
    return _task_list;
}

bool CartesianInterfaceImpl::getCurrentPose(const std::string& end_effector, Eigen::Affine3d& w_T_ee) const
{
    auto task = get_cart_task(end_effector);
    
    if(!task)
    {
        return false;
    }
    
    task->getCurrentPose(w_T_ee);
    
    return true;
    
}

const std::string& CartesianInterfaceImpl::getBaseLink(const std::string& ee_name) const
{
    auto task = get_cart_task(ee_name);
    
    if(!task)
    {
        throw std::invalid_argument("Undefined end effector");
    }
    
    return task->getBaseLink();
}

ControlType CartesianInterfaceImpl::getControlMode(const std::string& ee_name) const
{
    auto task = get_cart_task(ee_name);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return ControlType::Position;
    }
    
    return task->getControlMode();
}


bool CartesianInterfaceImpl::setControlMode(const std::string& ee_name, ControlType ctrl_type)
{
    auto task = get_cart_task(ee_name);
    
    if(!task)
    {
        return false;
    }
    
    task->setControlMode(ctrl_type);
    
    Logger::success(Logger::Severity::HIGH,
                    "Control mode changed to %s for task '%s' \n",
                    ControlTypeAsString(ctrl_type).c_str(), task->getName().c_str());

    return true;
}


State CartesianInterfaceImpl::getTaskState(const std::string& end_effector) const
{
    auto task = get_cart_task(end_effector);
    
    if(!task)
    {
        XBot::Logger::error("Undefined end effector \n");
        return State::Online;
    }
    
    return task->getTaskState();
}


XBot::ModelInterface::Ptr CartesianInterfaceImpl::getModel() const
{
    return _model;
}

const ProblemDescription &XBot::Cartesian::CartesianInterfaceImpl::getIkProblem() const
{
    return _ik_problem;
}

bool CartesianInterfaceImpl::getComPositionReference(Eigen::Vector3d& w_com_ref,
                                                     Eigen::Vector3d* base_vel_ref,
                                                     Eigen::Vector3d* base_acc_ref) const
{
    if(_com_task_map.empty())
    {
        Logger::error("Undefined task com\n");
        return false;
    }

    if(_com_task_map.size() > 1)
    {
        Logger::warning("Many com tasks defined, results might be inaccurate! \n");
    }

    auto com_task = _com_task_map.begin()->second;

    Eigen::Affine3d T;
    Eigen::Vector6d v, a;

    com_task->getPoseReference(T, &v, &a);
    
    w_com_ref = T.translation();
    if(base_vel_ref) *base_vel_ref = v.head<3>();
    if(base_acc_ref) *base_acc_ref = a.head<3>();
    
    return true;
}

bool CartesianInterfaceImpl::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
{
    if(_com_task_map.empty())
    {
        return false;
    }

    if(_com_task_map.size() > 1)
    {
        Logger::warning("Many com tasks defined, results might be inaccurate! \n");
    }
    
    Eigen::Affine3d Tgoal;

    _com_task_map.begin()->second->getPoseTarget(Tgoal);

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
    return !_postural_task_map.empty();
}


bool XBot::Cartesian::CartesianInterfaceImpl::getReferencePosture(Eigen::VectorXd& qref) const
{
    if(!postural_task_defined())
    {
        return false;
    }

    if(_postural_task_map.size() > 1)
    {
        Logger::warning("Many postural tasks defined, results might be inaccurate! \n");
    }
    
    _postural_task_map.begin()->second->getReferencePosture(qref);
    
    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::getReferencePosture(XBot::JointNameMap& qref) const
{
    if(!postural_task_defined())
    {
        return false;
    }

    if(_postural_task_map.size() > 1)
    {
        Logger::warning("Many postural tasks defined, results might be inaccurate! \n");
    }

    _postural_task_map.begin()->second->getReferencePosture(qref);

    return true;
}

bool XBot::Cartesian::CartesianInterfaceImpl::setReferencePosture(const XBot::JointNameMap& qref)
{
    if(!postural_task_defined())
    {
        return false;
    }

    if(_postural_task_map.size() > 1)
    {
        Logger::warning("Many postural tasks defined, results might be inaccurate! \n");
    }

    _postural_task_map.begin()->second->setReferencePosture(qref);

    return true;
}

void XBot::Cartesian::CartesianInterfaceImpl::enableOtg(double expected_dt)
{
    XBot::Logger::info(Logger::Severity::LOW, "Online trajectory generator enabled \n");
    
    for(auto tpair : _cart_task_map)
    {
        tpair.second->enableOnlineTrajectoryGeneration();
    }
}


void XBot::Cartesian::CartesianInterfaceImpl::setAccelerationLimits(const std::string& ee_name,
                                                                    double max_acc_lin,
                                                                    double max_acc_ang)
{
    auto task = get_cart_task(ee_name);
    
    if(!task)
    {
        return;
    }
    
    XBot::Logger::info(Logger::Severity::LOW, "Setting acceleration limits for task '%s' (lin: %f, ang: %f) \n",
                       ee_name.c_str(),
                       max_acc_lin,
                       max_acc_ang
                       );
    
    task->setAccelerationLimits(max_acc_lin, max_acc_ang);
}


void XBot::Cartesian::CartesianInterfaceImpl::setVelocityLimits(const std::string& ee_name,
                                                                double max_vel_lin,
                                                                double max_vel_ang)
{
    auto task = get_cart_task(ee_name);

    if(!task)
    {
        return;
    }

    XBot::Logger::info(Logger::Severity::LOW, "Setting acceleration limits for task '%s' (lin: %f, ang: %f) \n",
                       ee_name.c_str(),
                       max_vel_lin,
                       max_vel_ang
                       );

    task->setVelocityLimits(max_vel_lin, max_vel_ang);
}


void XBot::Cartesian::CartesianInterfaceImpl::getAccelerationLimits(const std::string& ee_name,
                                                                    double& max_acc_lin,
                                                                    double& max_acc_ang) const
{
    auto task = get_cart_task(ee_name);

    if(!task)
    {
        return;
    }

    task->getAccelerationLimits(max_acc_lin, max_acc_ang);
}

void CartesianInterfaceImpl::getVelocityLimits(const std::string& ee_name,
                                               double& max_vel_lin,
                                               double& max_vel_ang) const
{
    auto task = get_cart_task(ee_name);

    if(!task)
    {
        return;
    }

    task->getVelocityLimits(max_vel_lin, max_vel_ang);
}

bool CartesianInterfaceImpl::resetWorld(const Eigen::Affine3d& w_T_new_world)
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

TaskDescription::Ptr CartesianInterfaceImpl::getTask(const std::string & task_name)
{
    return _task_map.at(task_name);
}

bool CartesianInterfaceImpl::setDesiredDamping(const std::string& end_effector,
                                               const Eigen::Matrix6d& d)
{
    return false;
}

bool CartesianInterfaceImpl::setDesiredStiffness(const std::string& end_effector,
                                                 const Eigen::Matrix6d& k)
{
    return false;
}

bool CartesianInterfaceImpl::setForceReference(const std::string& end_effector,
                                               const Eigen::Vector6d& force)
{
    return false;
}

bool XBot::Cartesian::CartesianInterfaceImpl::getDesiredInteraction(const std::string& end_effector,
                                                                    Eigen::Vector6d& force,
                                                                    Eigen::Matrix6d& stiffness,
                                                                    Eigen::Matrix6d& damping) const
{
    return false;
}

CartesianInterfaceImpl::~CartesianInterfaceImpl()
{
    _logger->flush();
    Logger::success(Logger::Severity::HIGH, "Cleanly exiting from CartesI/O\n");
}

namespace XBot { namespace Cartesian {

template <>
std::string EnumToString<ActivationState>(ActivationState value)
{
    if(value == ActivationState::Enabled) return "Enabled";
    if(value == ActivationState::Disabled) return "Disabled";
    return "";
}

template <>
std::string EnumToString<ControlType>(ControlType value)
{
    if(value == ControlType::Position) return "Position";
    if(value == ControlType::Velocity) return "Velocity";
    return "";
}


template <>
std::string EnumToString<State>(State value)
{
    if(value == State::Reaching) return "Reaching";
    if(value == State::Online)   return "Online";
    return "";
}


template <>
ControlType StringToEnum<ControlType>(const std::string& value)
{
    std::string ctrl_lower = value;
    boost::algorithm::to_lower(ctrl_lower);

    if(ctrl_lower == "position") return ControlType::Position;
    if(ctrl_lower == "velocity") return ControlType::Velocity;

    throw std::invalid_argument("Invalid control type '" + value + "'");
}

template <>
State StringToEnum<State>(const std::string& value)
{
    std::string ctrl_lower = value;
    boost::algorithm::to_lower(ctrl_lower);

    if(ctrl_lower == "reaching") return State::Reaching;
    if(ctrl_lower == "online") return State::Online;

    throw std::invalid_argument("Invalid control type '" + value + "'");
}

template <>
ActivationState StringToEnum<ActivationState>(const std::string& value)
{
    std::string ctrl_lower = value;
    boost::algorithm::to_lower(ctrl_lower);

    if(ctrl_lower == "enabled")  return ActivationState::Enabled;
    if(ctrl_lower == "disabled") return ActivationState::Disabled;

    throw std::invalid_argument("Invalid control type '" + value + "'");
}

} }


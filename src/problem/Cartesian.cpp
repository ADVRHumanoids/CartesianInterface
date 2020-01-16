#include <cartesian_interface/problem/Cartesian.h>

using namespace XBot::Cartesian;

namespace
{
const double DEFAULT_TTL = 0.1;
const double DEFAULT_REACH_THRESHOLD = 1e-6;
}


CartesianTask::CartesianTask(ModelInterface::ConstPtr model,
                             std::string name,
                             std::string distal_link,
                             std::string base_link):
    TaskDescription("Cartesian", name, 6, model),
    _distal_link(distal_link),
    _base_link(base_link),
    _orientation_gain(1.0),
    _is_body_jacobian(false),
    _ctrl_mode(ControlType::Position),
    _state(State::Online),
    _vref_time_to_live(-1.0)
{
    _otg_maxvel.setConstant(1.0);
    _otg_maxacc.setConstant(10.0);
    _trajectory = std::make_shared<Trajectory>();

    reset();
}

namespace
{
std::string get_name(YAML::Node task_node)
{
    if(task_node["type"].as<std::string>() == "Com")
    {
        return "Com";
    }
    
    if(task_node["name"])
    {
        return task_node["name"].as<std::string>();
    }
    else
    {
        return task_node["distal_link"].as<std::string>();
    }
}

int get_size(YAML::Node task_node)
{
    if(task_node["type"].as<std::string>() == "Com")
    {
        return 3;
    }
    
    return 6;
}
}

CartesianTask::CartesianTask(YAML::Node task_node, ModelInterface::ConstPtr model):
    TaskDescription(task_node, model, ::get_name(task_node), ::get_size(task_node)),
    _ctrl_mode(ControlType::Position),
    _state(State::Online),
    _vref_time_to_live(-1.0),
    _orientation_gain(1.0),
    _is_body_jacobian(false)
{
    bool is_com = task_node["type"].as<std::string>() == "Com";
    
    _distal_link = is_com ? "com" : task_node["distal_link"].as<std::string>();
    _base_link = "world";
    
    if(task_node["base_link"])
    {
        _base_link = task_node["base_link"].as<std::string>();
    }
    
    if(task_node["orientation_gain"])
    {
        _orientation_gain = task_node["orientation_gain"].as<double>();
    }
    
    if(task_node["use_body_jacobian"] && task_node["use_body_jacobian"].as<bool>())
    {
        _is_body_jacobian = true;
    }

    _otg_maxvel.setConstant(1.0);
    _otg_maxacc.setConstant(10.0);
    _trajectory = std::make_shared<Trajectory>();

    reset();
}

bool CartesianTask::validate()
{
    bool ret = TaskDescription::validate();

    Eigen::Affine3d T;
    if(!_model->getPose(_base_link, T))
    {
        Logger::error("Task '%s': invalid base link '%s' \n",
                      getName().c_str(), _base_link.c_str());

        ret = false;
    }

    if(_distal_link != "com" && !_model->getPose(_distal_link, T))
    {
        Logger::error("Task '%s': invalid distal link '%s' \n",
                      getName().c_str(), _distal_link.c_str());

        ret = false;
    }

    if(_distal_link == "com" && _base_link != "world")
    {
        Logger::error("Task '%s': invalid com task with base link '%s' != 'world' \n",
                      getName().c_str(), _base_link.c_str());

        ret = false;
    }

    return ret;
}

void CartesianTask::getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const
{
    max_vel_lin = _otg_maxvel[0];
    max_vel_ang = _otg_maxvel[3]*2;
}

void CartesianTask::getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const
{
    max_acc_lin = _otg_maxacc[0];
    max_acc_ang = _otg_maxacc[3]*2;
}

void CartesianTask::setVelocityLimits(double max_vel_lin, double max_vel_ang)
{
    _otg_maxvel << max_vel_lin, max_vel_lin, max_vel_lin,
            max_vel_ang/2.0, max_vel_ang/2.0, max_vel_ang/2.0, max_vel_ang/2.0;
    
    if(_otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_vel_limits (%f, %f): success\n",
                        max_vel_lin,
                        max_vel_ang);
        
        _otg->setVelocityLimits(_otg_maxvel);
    }
    
    NOTIFY_OBSERVERS(SafetyLimits)
}

void CartesianTask::setAccelerationLimits(double max_acc_lin, double max_acc_ang)
{
    _otg_maxacc << max_acc_lin, max_acc_lin, max_acc_lin,
            max_acc_ang/2.0, max_acc_ang/2.0, max_acc_ang/2.0, max_acc_ang/2.0;
    
    if(_otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_acc_limits (%f, %f): success\n",
                        max_acc_lin,
                        max_acc_ang);
        
        _otg->setAccelerationLimits(_otg_maxacc);
    }
    
    NOTIFY_OBSERVERS(SafetyLimits)
}

void CartesianTask::enableOtg()
{
    _otg = std::make_shared<Reflexxes::Utils::TrajectoryGenerator>(7, // 3 + 4
                                                                  _ctx.getControlPeriod(),
                                                                  EigenVector7d::Zero());
    reset_otg();
    _otg->setVelocityLimits(_otg_maxvel);
    _otg->setAccelerationLimits(_otg_maxacc);
}

State CartesianTask::getTaskState() const
{
    return _state;
}

const std::string & CartesianTask::getBaseLink() const
{
    return _base_link;
}

bool CartesianTask::getPoseReference(Eigen::Affine3d & base_T_ref,
                                     Eigen::Vector6d * base_vel_ref,
                                     Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = get_pose_ref_otg();
    
    if(base_vel_ref) *base_vel_ref = _vel;
    if(base_acc_ref) *base_acc_ref = _acc;
    
    return true;
}

bool CartesianTask::getPoseReferenceRaw(Eigen::Affine3d & base_T_ref,
                                        Eigen::Vector6d * base_vel_ref,
                                        Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = _T;
    
    if(base_vel_ref) *base_vel_ref = _vel;
    if(base_acc_ref) *base_acc_ref = _acc;
    
    return true;
}

bool CartesianTask::getPoseTarget(Eigen::Affine3d & base_T_ref) const
{
    if(_state == State::Reaching)
    {
        base_T_ref = _trajectory->getWayPoints().back().frame;
        return true;
    }
    else
    {
        XBot::Logger::warning("Task '%s' is NOT in REACHING mode \n", _distal_link.c_str());
        return false;
    }
}

int CartesianTask::getCurrentSegmentId() const
{
    return _trajectory->getCurrentSegmentId(getTime());
}

bool CartesianTask::setBaseLink(const std::string & new_base_link)
{
    /* Check that new base link exists */
    Eigen::Affine3d T;
    if(!_model->getPose(new_base_link, T))
    {
        XBot::Logger::error("New base link '%s' in not defined\n",
                            new_base_link.c_str());
        return false;
    }
    
    /* Check that the task is not in reaching mode */
    if( _state != State::Online )
    {
        XBot::Logger::error("Task '%s': unable to change base link while performing a reach\n",
                            _distal_link.c_str());
        return false;
    }
    
    /* Update task */
    _base_link = new_base_link;
    _vel.setZero();
    _acc.setZero();
    
    /* Reset reference */
    reset();
    
    NOTIFY_OBSERVERS(BaseLink);
    
    return true;
}

const std::string & CartesianTask::getDistalLink() const
{
    return _distal_link;
}

bool CartesianTask::setPoseReference(const Eigen::Affine3d & base_T_ref)
{
    if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in REACHING state \n",
                            _distal_link.c_str());
        return false;
    }
    
    if(getActivationState() == ActivationState::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in DISABLED mode \n",
                            _distal_link.c_str());
        return false;
    }
    
    if(_ctrl_mode == ControlType::Position)
    {
        _T = base_T_ref;
    }
    else
    {
        Logger::warning(Logger::Severity::DEBUG,
                        "Task '%s': not setting position reference\n",
                        getName().c_str());
    }
    
    return true;
}

bool CartesianTask::setPoseReferenceRaw(const Eigen::Affine3d & base_T_ref)
{
    bool ret = setPoseReference(base_T_ref);
    reset_otg();
    
    return ret;
}

bool CartesianTask::setVelocityReference(const Eigen::Vector6d & base_vel_ref)
{
    if(getActivationState() == ActivationState::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in DISABLED mode \n",
                            getName().c_str());
        return false;
    }
    
    _vel = base_vel_ref;
    _vref_time_to_live = DEFAULT_TTL;
    
    return true;
}

bool CartesianTask::setPoseTarget(const Eigen::Affine3d & base_T_ref, double time)
{
    if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    if(_ctrl_mode != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in NOT in position mode \n",
                            getName().c_str());
        return false;
    }
    
    _state = State::Reaching;
    _trajectory->clear();
    _trajectory->addWayPoint(getTime(), _T);
    _trajectory->addWayPoint(getTime() + time, base_T_ref);
    _trajectory->compute();
    
    return true;
}

bool CartesianTask::setWayPoints(const Trajectory::WayPointVector & way_points)
{
    if(_state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    if(_ctrl_mode != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in NOT in position mode \n",
                            getName().c_str());
        return false;
    }
    
    _state = State::Reaching;
    _trajectory->clear();
    _trajectory->addWayPoint(getTime(), _T);
    
    for(const auto& wp : way_points)
    {
        _trajectory->addWayPoint(wp, getTime());
    }
    
    _trajectory->compute();
    return true;
}

bool CartesianTask::getCurrentPose(Eigen::Affine3d & base_T_ee) const
{
    base_T_ee = get_current_pose();
    return true;
}

void CartesianTask::abort()
{
    if(_state == State::Reaching)
    {
        _state = State::Online;
        _trajectory->clear();
    }
}

void CartesianTask::update(double time, double period)
{
    TaskDescription::update(time, period);

    _vref_time_to_live -= period;
    
    if(_state == State::Reaching)
    {
        _T = _trajectory->evaluate(time, &_vel, &_acc);

        if(_trajectory->isTrajectoryEnded(time) && check_reach())
        {
            _state = State::Online;
        }
    }
    else
    {
        if(_vref_time_to_live <= 0.0)
        {
            _vel.setZero();
            _acc.setZero();
            _vref_time_to_live = -1.0;
        }
    }
    
    apply_otg();
}

void CartesianTask::reset()
{
    _T = get_current_pose();
    
    _vel.setZero();
    _acc.setZero();
    
    _state = State::Online;
    
    reset_otg();
}

void CartesianTask::log(MatLogger::Ptr logger, bool init_logger, int buf_size)
{
    if(init_logger)
    {
        logger->createVectorVariable(getName() + "_pos",     3, buf_size);
        logger->createVectorVariable(getName() + "_pos_otg", 3, buf_size);
        logger->createVectorVariable(getName() + "_vel",     6, buf_size);
        logger->createVectorVariable(getName() + "_rot",     4, buf_size);
        logger->createVectorVariable(getName() + "_rot_otg", 4, buf_size);
        logger->createScalarVariable(getName() + "_state",      buf_size);
        return;
    }

    logger->add(getName() + "_pos",     _T.translation());
    logger->add(getName() + "_pos_otg", get_pose_ref_otg().translation());
    logger->add(getName() + "_vel",     _vel);
    logger->add(getName() + "_rot",     Eigen::Quaterniond(_T.linear()).coeffs());
    logger->add(getName() + "_rot_otg", Eigen::Quaterniond(get_pose_ref_otg().linear()).coeffs());
    logger->add(getName() + "_state",   _state == State::Reaching);
}

void CartesianTask::registerObserver(CartesianTaskObserver::WeakPtr observer)
{
    _observers.push_back(observer);
}

ControlType CartesianTask::getControlMode() const
{
    return _ctrl_mode;
}

bool CartesianTask::setControlMode(const ControlType & value)
{
    _ctrl_mode = value;
    
    NOTIFY_OBSERVERS(ControlMode);
    
    return true;
}

bool CartesianTask::check_reach() const
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    auto Totg = get_pose_ref_otg();
    auto Tref = _trajectory->getWayPoints().back().frame;
    
    return Tref.isApprox(Totg, DEFAULT_REACH_THRESHOLD);
}

void CartesianTask::apply_otg()
{
    if(!_otg) return;
    
    // TBD: also velocity level
    Eigen::Quaterniond q_actual(_otg_des.tail<4>());
    Eigen::Quaterniond q_des(_T.linear());
    Eigen::Vector4d q_des_coeffs = q_des.coeffs();
    if(q_actual.dot(q_des) < 0)
    {
        q_des_coeffs = -q_des_coeffs;
    }
    
    _otg_des << _T.translation(), q_des_coeffs;
    
    _otg->setReference(_otg_des, EigenVector7d::Zero());
    _otg->update(_otg_ref, _otg_vref);
    
    if(_otg_ref.tail<4>().norm() < 0.01)
    {
        Logger::warning("Experimental orientation otg: norm < 0.01 for task '%s': contact the developers! \n",
                        getName().c_str());
        
    }
    
}

void CartesianTask::reset_otg()
{
    if(!_otg) return;
    
    _otg_des << _T.translation(), Eigen::Quaterniond(_T.linear()).coeffs();
    _otg_ref = _otg_des;
    
    
    _otg->reset(_otg_des);
}

Eigen::Affine3d CartesianTask::get_pose_ref_otg() const
{
    if(!_otg)
    {
        return _T;
    }
    
    Eigen::Affine3d Totg;
    Eigen::Quaterniond q(_otg_ref.tail<4>());
    q.normalize();
    
    Totg.setIdentity();
    
    Totg.translation() = _otg_ref.head<3>();
    Totg.linear() = q.toRotationMatrix();
    
    return Totg;
}

Eigen::Affine3d CartesianTask::get_current_pose() const
{
    Eigen::Affine3d ret;
    
    if(_distal_link == "com")
    {
        Eigen::Vector3d com;
        _model->getCOM(com);
        ret.setIdentity();
        ret.translation() = com;

        return ret;
    }
    
    if(_base_link == "world")
    {
        _model->getPose(_distal_link, ret);
    }
    else
    {
        _model->getPose(_distal_link, _base_link, ret);
    }
    
    return ret;
}

bool CartesianTask::isBodyJacobian() const
{
    return _is_body_jacobian;
}

bool CartesianTaskObserver::onBaseLinkChanged()
{
    return true;
}

bool CartesianTaskObserver::onControlModeChanged()
{
    return true;
}

bool CartesianTaskObserver::onSafetyLimitsChanged()
{
    return true;
}

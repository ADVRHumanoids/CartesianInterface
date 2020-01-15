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
    distal_link(distal_link),
    base_link(base_link),
    orientation_gain(1.0),
    is_body_jacobian(false),
    ctrl_mode(ControlType::Position)
{
    __otg_maxvel.setConstant(1.0);
    __otg_maxacc.setConstant(10.0);
    trajectory = std::make_shared<Trajectory>();
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
    ctrl_mode(ControlType::Position)
{
    bool is_com = task_node["type"].as<std::string>() == "Com";
    
    distal_link = is_com ? "com" : task_node["distal_link"].as<std::string>();
    base_link = "world";
    
    if(task_node["base_link"])
    {
        base_link = task_node["base_link"].as<std::string>();
    }
    
    if(task_node["orientation_gain"])
    {
        orientation_gain = task_node["orientation_gain"].as<double>();
    }
    
    if(task_node["use_body_jacobian"] && task_node["use_body_jacobian"].as<bool>())
    {
        is_body_jacobian = true;
    }

    __otg_maxvel.setConstant(1.0);
    __otg_maxacc.setConstant(10.0);
    trajectory = std::make_shared<Trajectory>();
}

bool CartesianTask::validate()
{
    bool ret = TaskDescription::validate();

    Eigen::Affine3d T;
    if(!_model->getPose(base_link, T))
    {
        Logger::error("Task '%s': invalid base link '%s' \n",
                      getName().c_str(), base_link.c_str());

        ret = false;
    }

    if(distal_link != "com" && !_model->getPose(distal_link, T))
    {
        Logger::error("Task '%s': invalid distal link '%s' \n",
                      getName().c_str(), distal_link.c_str());

        ret = false;
    }

    if(distal_link == "com" && base_link != "world")
    {
        Logger::error("Task '%s': invalid com task with base link '%s' != 'world' \n",
                      getName().c_str(), base_link.c_str());

        ret = false;
    }

    return ret;
}

void CartesianTask::getVelocityLimits(double & max_vel_lin, double & max_vel_ang) const
{
    max_vel_lin = __otg_maxvel[0];
    max_vel_ang = __otg_maxvel[3]*2;
}

void CartesianTask::getAccelerationLimits(double & max_acc_lin, double & max_acc_ang) const
{
    max_acc_lin = __otg_maxacc[0];
    max_acc_ang = __otg_maxacc[3]*2;
}

void CartesianTask::setVelocityLimits(double max_vel_lin, double max_vel_ang)
{
    __otg_maxvel << max_vel_lin, max_vel_lin, max_vel_lin,
            max_vel_ang/2.0, max_vel_ang/2.0, max_vel_ang/2.0, max_vel_ang/2.0;
    
    if(otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_vel_limits (%f, %f): success\n",
                        max_vel_lin,
                        max_vel_ang);
        
        otg->setVelocityLimits(__otg_maxvel);
    }
    
    NOTIFY_OBSERVERS(SafetyLimits)
}

void CartesianTask::setAccelerationLimits(double max_acc_lin, double max_acc_ang)
{
    __otg_maxacc << max_acc_lin, max_acc_lin, max_acc_lin,
            max_acc_ang/2.0, max_acc_ang/2.0, max_acc_ang/2.0, max_acc_ang/2.0;
    
    if(otg)
    {
        Logger::success(Logger::Severity::LOW, "Task::set_otg_acc_limits (%f, %f): success\n",
                        max_acc_lin,
                        max_acc_ang);
        
        otg->setAccelerationLimits(__otg_maxacc);
    }
    
    NOTIFY_OBSERVERS(SafetyLimits)
}

void CartesianTask::enableOtg()
{
    otg = std::make_shared<Reflexxes::Utils::TrajectoryGenerator>(7, // 3 + 4
                                                                  ctx.getControlPeriod(),
                                                                  EigenVector7d::Zero());
    reset_otg();
    otg->setVelocityLimits(__otg_maxvel);
    otg->setAccelerationLimits(__otg_maxacc);
}

State CartesianTask::getTaskState() const
{
    return state;
}

const std::string & CartesianTask::getBaseLink() const
{
    return base_link;
}

bool CartesianTask::getPoseReference(Eigen::Affine3d & base_T_ref,
                                     Eigen::Vector6d * base_vel_ref,
                                     Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = get_pose_ref_otg();
    
    if(base_vel_ref) *base_vel_ref = vel;
    if(base_acc_ref) *base_acc_ref = acc;
    
    return true;
}

bool CartesianTask::getPoseReferenceRaw(Eigen::Affine3d & base_T_ref,
                                        Eigen::Vector6d * base_vel_ref,
                                        Eigen::Vector6d * base_acc_ref) const
{
    base_T_ref = T;
    
    if(base_vel_ref) *base_vel_ref = vel;
    if(base_acc_ref) *base_acc_ref = acc;
    
    return true;
}

bool CartesianTask::getPoseTarget(Eigen::Affine3d & base_T_ref) const
{
    if(state == State::Reaching)
    {
        base_T_ref = trajectory->getWayPoints().back().frame;
        return true;
    }
    else
    {
        XBot::Logger::warning("Task '%s' is NOT in REACHING mode \n", distal_link.c_str());
        return false;
    }
}

int CartesianTask::getCurrentSegmentId() const
{
    return trajectory->getCurrentSegmentId(getTime());
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
    if( state != State::Online )
    {
        XBot::Logger::error("Task '%s': unable to change base link while performing a reach\n",
                            distal_link.c_str());
        return false;
    }
    
    /* Update task */
    base_link = new_base_link;
    vel.setZero();
    acc.setZero();
    
    /* Reset reference */
    reset();
    
    NOTIFY_OBSERVERS(BaseLink);
    
    return true;
}

const std::string & CartesianTask::getDistalLink() const
{
    return distal_link;
}

bool CartesianTask::setPoseReference(const Eigen::Affine3d & base_T_ref)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in REACHING state \n",
                            distal_link.c_str());
        return false;
    }
    
    if(getActivationState() == ActivationState::Disabled)
    {
        XBot::Logger::error("Unable to set pose reference. Task '%s' is in DISABLED mode \n",
                            distal_link.c_str());
        return false;
    }
    
    if(ctrl_mode == ControlType::Position)
    {
        T = base_T_ref;
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
    
    vel = base_vel_ref;
    vref_time_to_live = DEFAULT_TTL;
    
    return true;
}

bool CartesianTask::setPoseTarget(const Eigen::Affine3d & base_T_ref, double time)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    if(ctrl_mode != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in NOT in position mode \n",
                            getName().c_str());
        return false;
    }
    
    state = State::Reaching;
    trajectory->clear();
    trajectory->addWayPoint(getTime(), T);
    trajectory->addWayPoint(getTime() + time, base_T_ref);
    trajectory->compute();
    
    return true;
}

bool CartesianTask::setWayPoints(const Trajectory::WayPointVector & way_points)
{
    if(state == State::Reaching)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in already in REACHING mode \n",
                            getName().c_str());
        return false;
    }
    
    if(ctrl_mode != ControlType::Position)
    {
        XBot::Logger::error("Unable to set target pose. Task '%s' is in NOT in position mode \n",
                            getName().c_str());
        return false;
    }
    
    state = State::Reaching;
    trajectory->clear();
    trajectory->addWayPoint(getTime(), T);
    
    for(const auto& wp : way_points)
    {
        trajectory->addWayPoint(wp, getTime());
    }
    
    trajectory->compute();
    return true;
}

bool CartesianTask::getCurrentPose(Eigen::Affine3d & base_T_ee) const
{
    base_T_ee = get_current_pose();
    return true;
}

void CartesianTask::abort()
{
    if(state == State::Reaching)
    {
        state = State::Online;
        trajectory->clear();
    }
}

void CartesianTask::update(double time, double period)
{
    TaskDescription::update(time, period);

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
}

void CartesianTask::reset()
{
    T = get_current_pose();
    
    vel.setZero();
    acc.setZero();
    
    state = State::Online;
    
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
    }

    logger->add(getName() + "_pos",     T.translation());
    logger->add(getName() + "_pos_otg", get_pose_ref_otg().translation());
    logger->add(getName() + "_vel",     vel);
    logger->add(getName() + "_rot",     Eigen::Quaterniond(T.linear()).coeffs());
    logger->add(getName() + "_rot_otg", Eigen::Quaterniond(get_pose_ref_otg().linear()).coeffs());
    logger->add(getName() + "_state",   state == State::Reaching);
}

void CartesianTask::registerObserver(CartesianTaskObserver::WeakPtr observer)
{
    _observers.push_back(observer);
}

ControlType CartesianTask::getControlMode() const
{
    return ctrl_mode;
}

bool CartesianTask::setControlMode(const ControlType & value)
{
    ctrl_mode = value;
    
    NOTIFY_OBSERVERS(ControlMode);
    
    return true;
}

bool CartesianTask::check_reach() const
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    auto Totg = get_pose_ref_otg();
    auto Tref = trajectory->getWayPoints().back().frame;
    
    return Tref.isApprox(Totg, DEFAULT_REACH_THRESHOLD);
}

void CartesianTask::apply_otg()
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
        Logger::warning("Experimental orientation otg: norm < 0.01 for task '%s': contact the developers! \n",
                        getName().c_str());
        
    }
    
}

void CartesianTask::reset_otg()
{
    if(!otg) return;
    
    __otg_des << T.translation(), Eigen::Quaterniond(T.linear()).coeffs();
    __otg_ref = __otg_des;
    
    
    otg->reset(__otg_des);
}

Eigen::Affine3d CartesianTask::get_pose_ref_otg() const
{
    if(!otg)
    {
        return T;
    }
    
    Eigen::Affine3d Totg;
    Eigen::Quaterniond q(__otg_ref.tail<4>());
    q.normalize();
    
    Totg.setIdentity();
    
    Totg.translation() = __otg_ref.head<3>();
    Totg.linear() = q.toRotationMatrix();
    
    return Totg;
}

Eigen::Affine3d CartesianTask::get_current_pose() const
{
    Eigen::Affine3d ret;
    
    if(distal_link == "com")
    {
        Eigen::Vector3d com;
        _model->getCOM(com);
        ret.setIdentity();
        ret.translation() = com;

        return ret;
    }
    
    if(base_link == "world")
    {
        _model->getPose(distal_link, ret);
    }
    else
    {
        _model->getPose(distal_link, base_link, ret);
    }
    
    return ret;
}


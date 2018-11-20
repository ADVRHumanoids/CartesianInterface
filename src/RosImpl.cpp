#include <cartesian_interface/ros/RosImpl.h>

#include <cartesian_interface/GetTaskList.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/LoadController.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#define THROW_NOT_IMPL throw std::runtime_error("Not implemented function " + std::string(__func__));

using namespace XBot::Cartesian;

void XBot::Cartesian::RosImpl::construct_from_tasklist()
{
    getTaskList();
    
    _task_map.clear();
    for(auto t: _tasklist)
    {
        ROS_INFO("Task %s added to Cartesian interface", t.c_str());
        _task_map[t] = std::make_shared<RosTask>(_nh, t);
    }
    
    int n_attempt = 0;
    bool is_valid = false;
    const int MAX_ATTEMPT = 100;
    
    while(!is_valid && n_attempt++ < MAX_ATTEMPT)
    {
        _queue.callAvailable();
        is_valid = true;
        for(auto& rt : _task_map)
        {
            is_valid &= rt.second->is_valid();
        }
        
        ros::Duration(0.01).sleep();
    }
    
    if(!is_valid)
    {
        throw std::runtime_error("Unable to receive valid state");
    }
}


RosImpl::RosImpl():
    _nh("cartesian")
{
    _nh.setCallbackQueue(&_queue);
    
    _tasklist_srv = _nh.serviceClient<cartesian_interface::GetTaskList>("get_task_list");
    
    construct_from_tasklist();
    
    _posture_pub = _nh.advertise<sensor_msgs::JointState>("posture/reference", 1);
    _load_ctrl_srv = _nh.serviceClient<cartesian_interface::LoadController>("load_controller");
    _upd_limits_srv = _nh.serviceClient<std_srvs::Trigger>("update_velocity_limits");
    
}

RosImpl::RosTask::RosTask(ros::NodeHandle nh, std::string arg_distal_link):
    distal_link(arg_distal_link),
    valid_state(false),
    reach_action(nh, arg_distal_link + "/reach")
{
    state_sub = nh.subscribe(distal_link + "/state", 1, &RosTask::state_callback, this);
    ref_pub = nh.advertise<geometry_msgs::PoseStamped>(distal_link + "/reference", 1);
    vref_pub = nh.advertise<geometry_msgs::TwistStamped>(distal_link + "/velocity_reference", 1);
    get_prop_srv = nh.serviceClient<cartesian_interface::GetTaskInfo>(distal_link + "/get_task_properties");
    set_prop_srv = nh.serviceClient<cartesian_interface::SetTaskInfo>(distal_link + "/set_task_properties");
    
}

void XBot::Cartesian::RosImpl::shutdown()
{
    std::cout << __func__ << std::endl;
    this->~RosImpl();
    std::cout << "Shutdown complete" << std::endl;
}


RosImpl::RosTask::Ptr RosImpl::get_task(const std::string& task_name, bool no_throw) const
{
    auto it = _task_map.find(task_name);
    
    if(it != _task_map.end())
    {
        return it->second;
    }
    else
    {
        if(!no_throw)
        {
            throw std::out_of_range("Task " + task_name + " undefined");
        }
        return nullptr;
    }
}


void RosImpl::RosTask::state_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    tf::poseMsgToEigen(msg->pose, state);
    valid_state = true;
}

bool RosImpl::RosTask::is_valid() const
{
    return valid_state;
}


const std::vector< std::string >& RosImpl::getTaskList() const
{
    if(!_tasklist_srv.waitForExistence(ros::Duration(1.0)))
    {
        _tasklist_srv.getService();
        throw std::runtime_error("Server for service " + _tasklist_srv.getService() + " is not running");
    }
    
    cartesian_interface::GetTaskList srv;
    
    if(!_tasklist_srv.call(srv))
    {
        throw std::runtime_error("Unable to call service " + _tasklist_srv.getService());
    }
    
    _tasklist.clear();
    for(auto dl : srv.response.distal_links)
    {
        _tasklist.push_back(dl.data);
    }
    
    return _tasklist;
}

bool RosImpl::update(double time, double period)
{
    _queue.callAvailable();
    return true;
}

Eigen::Affine3d XBot::Cartesian::RosImpl::RosTask::get_state() const
{
    return state;
}

void XBot::Cartesian::RosImpl::RosTask::get_properties(std::string& base_link, 
                                                       CartesianInterface::ControlType& ctrl_type)
{
    cartesian_interface::GetTaskInfo srv;
    if(!get_prop_srv.call(srv))
    {
        throw std::runtime_error("Unable to get properties for task " + distal_link);
    }
    
    base_link = srv.response.base_link;
    ctrl_type = ControlTypeFromString(srv.response.control_mode);
    
}

void XBot::Cartesian::RosImpl::RosTask::set_base_link(const std::string& base_link)
{
    cartesian_interface::SetTaskInfo srv;
    srv.request.base_link = base_link;
    if(!set_prop_srv.call(srv))
    {
        throw std::runtime_error("Unable to set properties for task " + distal_link);
    }
    if(!srv.response.success)
    {
        throw std::runtime_error("Service " + set_prop_srv.getService() + 
            " returned an error: \n\t" + srv.response.message);
    }
    
    ROS_INFO("%s", srv.response.message.c_str());
}

void XBot::Cartesian::RosImpl::RosTask::set_ctrl_mode(CartesianInterface::ControlType ctrl_type)
{
    cartesian_interface::SetTaskInfo srv;
    srv.request.control_mode = ControlTypeAsString(ctrl_type);
    if(!set_prop_srv.call(srv))
    {
        throw std::runtime_error("Unable to set properties for task " + distal_link);
    }
    if(!srv.response.success)
    {
        throw std::runtime_error("Service " + set_prop_srv.getService() + 
            " returned an error: \n\t" + srv.response.message);
    }
    
    ROS_INFO("%s", srv.response.message.c_str());
}


void XBot::Cartesian::RosImpl::RosTask::send_ref(const Eigen::Affine3d& ref)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    tf::poseEigenToMsg(ref, msg.pose);
    ref_pub.publish(msg);
}

void XBot::Cartesian::RosImpl::RosTask::send_vref(const Eigen::Vector6d& vref)
{
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    tf::twistEigenToMsg(vref, msg.twist);
    vref_pub.publish(msg);
}

void XBot::Cartesian::RosImpl::RosTask::send_waypoints(const Trajectory::WayPointVector& wp, bool incr)
{
    cartesian_interface::ReachPoseGoal goal;
    goal.incremental = incr;
    
    for(auto f : wp)
    {
        geometry_msgs::Pose frame;
        tf::poseEigenToMsg(f.frame, frame);
        
        goal.frames.push_back(frame);
        goal.time.push_back(f.time);
    }
 
    if(!reach_action.waitForServer(ros::Duration(2.0)))
    {
        throw std::runtime_error("Unable to connect to action server for task " + distal_link);
    }
    
    reach_action.sendGoal(goal);
    
}

bool XBot::Cartesian::RosImpl::RosTask::wait_for_result(ros::Duration timeout)
{
    return reach_action.waitForResult(timeout);
}

bool XBot::Cartesian::RosImpl::waitReachCompleted(const std::string& ee_name, double timeout_sec)
{
    auto task = get_task(ee_name, false);
    
    return task->wait_for_result(ros::Duration(timeout_sec));
}


bool RosImpl::getPoseReference(const std::string& end_effector, 
                                                Eigen::Affine3d& base_T_ref, 
                                                Eigen::Vector6d* base_vel_ref, 
                                                Eigen::Vector6d* base_acc_ref) const
{
    auto task = get_task(end_effector, false);
    
    if(!task)
    {
        return false;
    }
    
    base_T_ref = task->get_state();
    
    if(base_vel_ref) base_vel_ref->setZero();
    if(base_acc_ref) base_acc_ref->setZero();
    
    return true;
}

void XBot::Cartesian::RosImpl::loadController(const std::string& controller_name)
{
    cartesian_interface::LoadController srv;
    srv.request.controller_name = controller_name;
    srv.request.force_reload = false;
    
    _task_map.clear();
    
    if(!_load_ctrl_srv.call(srv))
    {
        throw std::runtime_error("Unable to connect to " + _load_ctrl_srv.getService());
    }
    
    if(!srv.response.success)
    {
        throw std::runtime_error(_load_ctrl_srv.getService() + " responded with an error:\n\t" + srv.response.message);
    }
    
    ROS_INFO("%s", srv.response.message.c_str());
    
    construct_from_tasklist();
}


RosImpl::~RosImpl()
{

}

bool RosImpl::abort(const std::string& end_effector)
{
    THROW_NOT_IMPL
}

const std::string& RosImpl::getBaseLink(const std::string& ee_name) const
{
    auto task = get_task(ee_name, false);
    
    ControlType ctrl_tmp;
    static std::string base_link_static; // NOTE: user must copy the output
    task->get_properties(base_link_static, ctrl_tmp);
    
    return base_link_static;
        
}

CartesianInterface::ControlType RosImpl::getControlMode(const std::string& ee_name) const
{
    auto task = get_task(ee_name, false);
    
    ControlType ctrl_tmp;
    std::string base_link_tmp;
    task->get_properties(base_link_tmp, ctrl_tmp);
    
    return ctrl_tmp;
}

bool RosImpl::getCurrentPose(const std::string& end_effector, Eigen::Affine3d& base_T_ee) const
{
    THROW_NOT_IMPL
}

bool RosImpl::getPoseTarget(const std::string& end_effector, Eigen::Affine3d& base_T_ref) const
{
    THROW_NOT_IMPL
}

CartesianInterface::State RosImpl::getTaskState(const std::string& end_effector) const
{
    THROW_NOT_IMPL
}

bool RosImpl::reset()
{
    THROW_NOT_IMPL
}

bool RosImpl::setComPositionReference(const Eigen::Vector3d& base_com_ref, const Eigen::Vector3d& base_vel_ref, const Eigen::Vector3d& base_acc_ref)
{
    THROW_NOT_IMPL
}

bool RosImpl::setControlMode(const std::string& ee_name, CartesianInterface::ControlType ctrl_type)
{
    auto task = get_task(ee_name, false);
    
    task->set_ctrl_mode(ctrl_type);
    
    return true;
}

bool RosImpl::setPoseReference(const std::string& end_effector, 
                               const Eigen::Affine3d& base_T_ref, 
                               const Eigen::Vector6d& base_vel_ref, 
                               const Eigen::Vector6d& base_acc_ref)
{
    auto task = get_task(end_effector, false);
    
    task->send_ref(base_T_ref);
    task->send_vref(base_vel_ref);
    
    return true;
}

bool RosImpl::setPositionReference(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, const Eigen::Vector3d& base_vel_ref, const Eigen::Vector3d& base_acc_ref)
{
    THROW_NOT_IMPL
}

bool RosImpl::setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time)
{
    THROW_NOT_IMPL
}

bool RosImpl::setTargetOrientation(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time)
{
    THROW_NOT_IMPL
}

bool RosImpl::setTargetOrientation(const std::string& end_effector, const std::string& base_frame, const Eigen::Matrix3d& base_R_ref, double time)
{
    THROW_NOT_IMPL
}

bool RosImpl::setTargetPose(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, double time)
{
    return setTargetPose(end_effector, base_T_ref, time, false);
}

bool RosImpl::setTargetPose(const std::string& end_effector, 
                            const Eigen::Affine3d& base_T_ref, 
                            double time, 
                            bool incremental)
{
    Trajectory::WayPointVector wpv;
    wpv.emplace_back(base_T_ref, time);
    return setWayPoints(end_effector, wpv, incremental);
}

bool RosImpl::setWayPoints(const std::string& end_effector, 
                           const Trajectory::WayPointVector& way_points, 
                           bool incremental)
{
    auto task = get_task(end_effector, false);
    
    task->send_waypoints(way_points, incremental);
    
    return true;
}


bool RosImpl::setTargetPosition(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time)
{
    THROW_NOT_IMPL
}

void RosImpl::getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const
{
    get_task(ee_name, false);
    
    if(!_nh.getParam(ee_name + "/max_acceleration_linear", max_acc_lin) || 
       !_nh.getParam(ee_name + "/max_acceleration_angular", max_acc_ang))
    {
        throw std::runtime_error("Undefine acceleration limits for task " + ee_name);
    }
}

bool RosImpl::getComPositionReference(Eigen::Vector3d& w_com_ref, Eigen::Vector3d* base_vel_ref, Eigen::Vector3d* base_acc_ref) const
{
    THROW_NOT_IMPL
}

bool RosImpl::getReferencePosture(Eigen::VectorXd& qref) const
{
    THROW_NOT_IMPL
}

bool RosImpl::getReferencePosture(XBot::JointNameMap& qref) const
{
    THROW_NOT_IMPL
}

void RosImpl::getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const
{
    get_task(ee_name, false);
    
    if(!_nh.getParam(ee_name + "/max_velocity_linear", max_vel_lin) || 
       !_nh.getParam(ee_name + "/max_velocity_angular", max_vel_ang))
    {
        throw std::runtime_error("Undefine velocity limits for task " + ee_name);
    }
}

bool RosImpl::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    THROW_NOT_IMPL
}

void RosImpl::setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang)
{
    double dummy;
    getAccelerationLimits(ee_name, dummy, dummy);
    
    _nh.setParam(ee_name + "/max_acceleration_linear",  max_acc_lin);
    _nh.setParam(ee_name + "/max_acceleration_angular", max_acc_ang);
        
    std_srvs::Trigger srv;
    
    if(!_upd_limits_srv.call(srv))
    {
        throw std::runtime_error("Unable to connect to " + _upd_limits_srv.getService());
    }
    
    if(!srv.response.success)
    {
        throw std::runtime_error(_upd_limits_srv.getService() + " responded with an error:\n\t" + srv.response.message);
    }
    
    ROS_INFO("%s", srv.response.message.c_str());
}

void RosImpl::setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang)
{
    double dummy;
    getVelocityLimits(ee_name, dummy, dummy);
    
    _nh.setParam(ee_name + "/max_velocity_linear",  max_vel_lin);
    _nh.setParam(ee_name + "/max_velocity_angular", max_vel_ang);
        
    std_srvs::Trigger srv;
    
    if(!_upd_limits_srv.call(srv))
    {
        throw std::runtime_error("Unable to connect to " + _upd_limits_srv.getService());
    }
    
    if(!srv.response.success)
    {
        throw std::runtime_error(_upd_limits_srv.getService() + " responded with an error:\n\t" + srv.response.message);
    }
    
    ROS_INFO("%s", srv.response.message.c_str());
}

bool RosImpl::reset(double time)
{
    THROW_NOT_IMPL
}

bool RosImpl::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
{
    THROW_NOT_IMPL
}

bool RosImpl::setWayPoints(const std::string& end_effector, const Trajectory::WayPointVector& way_points)
{
    return setWayPoints(end_effector, way_points, false);
}

bool RosImpl::setReferencePosture(const XBot::JointNameMap& qref)
{
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    
    for(auto p : qref)
    {
        msg.name.push_back(p.first);
        msg.position.push_back(p.second);
    }
    
    _posture_pub.publish(msg);
    
    return true;
}

bool RosImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
{
    auto task = get_task(ee_name, false);
    
    task->set_base_link(new_base_link);
    
    return true;
}

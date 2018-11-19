#include <cartesian_interface/ros/RosImpl.h>

#define THROW_NOT_IMPL throw std::runtime_error("Not implemented function " + std::string(__func__));

using namespace XBot::Cartesian;


RosImpl::RosImpl():
    _nh("cartesian")
{
    _nh.setCallbackQueue(&_queue);
    
    _tasklist_srv = _nh.serviceClient<cartesian_interface::GetTaskList>("get_task_list");
    
    getTaskList();
    
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

RosImpl::RosTask::RosTask(ros::NodeHandle nh, std::string arg_distal_link):
    distal_link(arg_distal_link),
    valid_state(false)
{
    state_sub = nh.subscribe(distal_link + "/state", 1, &RosTask::state_callback, this);
    ref_pub = nh.advertise<geometry_msgs::PoseStamped>(distal_link + "/reference", 1);
    vref_pub = nh.advertise<geometry_msgs::TwistStamped>(distal_link + "/velocity_reference", 1);
    prop_srv = nh.serviceClient<cartesian_interface::GetTaskInfo>(distal_link + "/get_task_properties");
    
    
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
    
    ROS_INFO("Received state %s", distal_link.c_str());
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

bool XBot::Cartesian::RosImpl::RosTask::get_properties(std::string& base_link, 
                                                       CartesianInterface::ControlType& ctrl_type)
{
    cartesian_interface::GetTaskInfo srv;
    if(!prop_srv.call(srv))
    {
        return false;
    }
    
    base_link = srv.response.base_link;
    ctrl_type = ControlTypeFromString(srv.response.control_mode);
    
    return true;
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
    static std::string base_link_static;
    if(!task->get_properties(base_link_static, ctrl_tmp))
    {
        throw std::runtime_error("Unable to get properties for task " + ee_name);
    }
    
    return base_link_static;
        
}

CartesianInterface::ControlType RosImpl::getControlMode(const std::string& ee_name) const
{
    auto task = get_task(ee_name, false);
    
    ControlType ctrl_tmp;
    static std::string base_link_static;
    if(!task->get_properties(base_link_static, ctrl_tmp))
    {
        throw std::runtime_error("Unable to get properties for task " + ee_name);
    }
    
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
    THROW_NOT_IMPL
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
    THROW_NOT_IMPL
}

bool RosImpl::setTargetPosition(const std::string& end_effector, const Eigen::Vector3d& base_pos_ref, double time)
{
    THROW_NOT_IMPL
}

void RosImpl::getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const
{
    THROW_NOT_IMPL
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
    THROW_NOT_IMPL
}

bool RosImpl::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    THROW_NOT_IMPL
}

void RosImpl::setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang)
{
    THROW_NOT_IMPL
}

void RosImpl::setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang)
{
    THROW_NOT_IMPL
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
    THROW_NOT_IMPL
}

bool RosImpl::setReferencePosture(const XBot::JointNameMap& qref)
{
    THROW_NOT_IMPL
}

bool RosImpl::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
{
    THROW_NOT_IMPL
}

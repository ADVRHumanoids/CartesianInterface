#include <cartesian_interface/ros/RosClient.h>
#include <cartesian_interface/ResetWorld.h>
#include <cartesian_interface/LoadController.h>
#include <cartesian_interface/GetTaskList.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_srvs/Trigger.h>

#include "fmt/format.h"

#include "ros/client_api/TaskRos.h"
#include "ros/client_api/CartesianRos.h"

#define THROW_NOT_IMPL throw std::runtime_error("Not implemented function " + std::string(__func__));

using namespace XBot::Cartesian;
using namespace cartesian_interface;

namespace
{

ProblemDescription construct_problem(ros::NodeHandle nh)
{

    auto get_task_list_srv = nh.serviceClient<GetTaskList>("get_task_list");


    int attempts = 100;
    while(attempts-- && !get_task_list_srv.exists())
    {
        if(attempts % 10 == 0)
        {
            fmt::print("Trying to contact server for service '{}' ... \n",
                       get_task_list_srv.getService());
        }

        usleep(0.1 * 1e6);
    }

    GetTaskList srv_list;
    if(!get_task_list_srv.waitForExistence(ros::Duration(1.0)) ||
            !get_task_list_srv.call(srv_list))
    {
        throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                             get_task_list_srv.getService()));
    }

    AggregatedTask tasks;

    for(int i = 0; i < srv_list.response.names.size(); i++)
    {
        std::string name = srv_list.response.names[i];

        auto get_task_info_srv = nh.serviceClient<GetTaskInfo>(name + "/get_task_properties");

        GetTaskInfo srv_info;
        if(!get_task_info_srv.waitForExistence(ros::Duration(1.0)) ||
                !get_task_info_srv.call(srv_info))
        {
            throw std::runtime_error(fmt::format("Unable to call service '{}'",
                                                 get_task_info_srv.getService()));
        }

        auto t = ClientApi::TaskRos::MakeInstance(name,
                                                  srv_info.response.type,
                                                  nh);

        tasks.push_back(t);

    }

    ProblemDescription ik_pb(tasks);

    attempts = 100;
    while(attempts-- && !ik_pb.validate())
    {
        if(attempts % 10 == 0) fmt::print("Waiting for all tasks to become valid... \n");

        auto queue = static_cast<ros::CallbackQueue*>(nh.getCallbackQueue());
        queue->callAvailable();
        usleep(0.1 * 1e6);
    }

    if(attempts == 0)
    {
        ik_pb.validate(true);
    }

    return ik_pb;

}

}

std::ostream& XBot::Cartesian::operator<<(std::ostream& os, const RosClient& r)
{
    os << "CartesianInterfaceRos running inside ROS node " << ros::this_node::getName() << "\n";
    auto tasklist = r.getTaskList();
    os << "Defined tasks: \n";
    for(auto t : tasklist)
    {
        os << " - ";
        os << XBot::bold_on << t << XBot::bold_off << "\n";
    }
    
    return os;
    
}

RosInitializer::RosInitializer(std::string ns)
{
    if(!ros::ok())
    {
        std::string ns_arg = "__ns:=";
        ns_arg += "";
        std::vector<const char *> args {"", ns_arg.c_str()};

        int argc = args.size();

        ros::init(argc, (char **)args.data(), "cartesio_ros",
                  ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

        ROS_WARN("Initializing roscpp under namespace '%s' with anonymous name '%s'",
                 ros::this_node::getNamespace().c_str(),
                 ros::this_node::getName().c_str()
                 );
    }

    _nh.reset(new ros::NodeHandle(ns));
    _nh->setCallbackQueue(&_queue);
}

ros::NodeHandle & RosInitializer::nh()
{
    return *_nh;
}

void RosInitializer::callAvailable()
{
    _queue.callAvailable();
}


RosClient::RosClient(std::string ns):
    RosInitializer(ns),
    CartesianInterfaceImpl(XBot::ModelInterface::Ptr(),
                           ::construct_problem(nh()))
{
    _load_ctrl_srv = nh().serviceClient<LoadController>("load_controller");
}

void RosClient::set_async_mode(bool async)
{
    for(auto t : getTaskList())
    {
        if(auto tros = std::dynamic_pointer_cast<ClientApi::TaskRos>(getTask(t)))
        {
            tros->setAsyncMode(async);
        }
    }
}

bool RosClient::setVelocityReference(const std::string& end_effector,
                                     const Eigen::Vector6d& base_vel_ref,
                                     const std::string& base_frame)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(end_effector));

    if(!cart_ros) return false;

    return cart_ros->setVelocityReference(base_vel_ref, base_frame);
}

bool RosClient::setWayPoints(const std::string& end_effector,
                             const Trajectory::WayPointVector& way_points,
                             bool incremental)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(end_effector));

    if(!cart_ros) return false;

    return cart_ros->setWayPoints(way_points, incremental);
}

bool XBot::Cartesian::RosClient::getPoseFromTf(const std::string& source_frame,
                                               const std::string& target_frame,
                                               Eigen::Affine3d& t_T_s)
{

    tf::StampedTransform T;


    if(!_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0)))
    {
        ROS_ERROR("Wait for transform timed out");
        return false;
    }


    try
    {
        _listener.lookupTransform(target_frame, source_frame, ros::Time(0), T);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    tf::transformTFToEigen(T, t_T_s);
    return true;
}




//void RosClient::construct_from_tasklist()
//{
//    getTaskList();

//    _task_map.clear();
//    _inter_task_map.clear();
//    for(auto t: _tasklist)
//    {
//        ROS_INFO("Task %s added to Cartesian interface", t.c_str());

//        auto task = std::make_shared<RosTask>(_nh, t);
//        std::string str_tmp;
//        ControlType ctrl_tmp;
//        TaskInterface ifc;
//        task->get_properties(str_tmp, ctrl_tmp, ifc);

//        if(ifc == TaskInterface::Interaction)
//        {
//            auto itask = std::make_shared<RosInteractionTask>(_nh, t);
//            task = itask;
//            _inter_task_map[t] = itask;
//        }

//        _task_map[t] = task;
//    }

//    int n_attempt = 0;
//    bool is_valid = false;
//    const int MAX_ATTEMPT = 100;

//    while(!is_valid && n_attempt++ < MAX_ATTEMPT)
//    {
//        _queue.callAvailable();
//        is_valid = true;
//        for(auto& rt : _task_map)
//        {
//            is_valid &= rt.second->is_valid();
//        }

//        ros::Duration(0.05).sleep();
//    }

//    if(!is_valid)
//    {
//        throw std::runtime_error("Unable to receive valid state");
//    }
//}


//RosClient::RosClient(std::string ns):
//    _ros_init_helper(""),
//    _nh(ns)
//{
//    _nh.setCallbackQueue(&_queue);

//    _tasklist_srv = _nh.serviceClient<cartesian_interface::GetTaskList>("get_task_list");

//    construct_from_tasklist();

//    _posture_pub = _nh.advertise<sensor_msgs::JointState>("posture/reference", 1);
//    _load_ctrl_srv = _nh.serviceClient<cartesian_interface::LoadController>("load_controller");
//    _upd_limits_srv = _nh.serviceClient<std_srvs::Trigger>("update_velocity_limits");

//}

//RosClient::RosTask::RosTask(ros::NodeHandle a_nh,
//                          std::string arg_distal_link):
//    distal_link(arg_distal_link),
//    valid_state(false),
//    reach_action(a_nh, arg_distal_link + "/reach"),
//    vref_async(arg_distal_link + "/velocity_reference", ros::Duration(VREF_ASYNC_PERIOD)),
//    async_mode(false),
//    nh(a_nh)
//{
//    state_sub = a_nh.subscribe(distal_link + "/state", 1, &RosTask::state_callback, this);
//    get_prop_sub = a_nh.subscribe(distal_link + "/task_properties", 1, &RosTask::task_properties_callback, this);
//    ref_pub = a_nh.advertise<geometry_msgs::PoseStamped>(distal_link + "/reference", 1);
//    vref_pub = a_nh.advertise<geometry_msgs::TwistStamped>(distal_link + "/velocity_reference", 1);
//    get_prop_srv = a_nh.serviceClient<cartesian_interface::GetTaskInfo>(distal_link + "/get_task_properties");
//    set_prop_srv = a_nh.serviceClient<cartesian_interface::SetTaskInfo>(distal_link + "/set_task_properties");

//    get_prop_srv.waitForExistence(ros::Duration(1.0));
//    set_prop_srv.waitForExistence(ros::Duration(1.0));
//}


//RosClient::RosTask::Ptr RosClient::get_task(const std::string& task_name, bool no_throw) const
//{
//    auto it = _task_map.find(task_name);

//    if(it != _task_map.end())
//    {
//        return it->second;
//    }
//    else
//    {
//        if(!no_throw)
//        {
//            throw std::out_of_range("Task " + task_name + " undefined");
//        }
//        return nullptr;
//    }
//}

//RosClient::RosInteractionTask::Ptr RosClient::get_interaction_task(const std::string& task_name,
//                                                                       bool no_throw) const
//{
//    auto it = _inter_task_map.find(task_name);

//    if(it != _inter_task_map.end())
//    {
//        return it->second;
//    }
//    else
//    {
//        if(!no_throw)
//        {
//            throw std::out_of_range("Interaction task " + task_name + " undefined");
//        }
//        return nullptr;
//    }
//}


//void RosClient::RosTask::task_properties_callback(const cartesian_interface::TaskInfoConstPtr & msg)
//{
//    cached_task_prop = *msg;
//}

//void RosClient::RosTask::state_callback(const geometry_msgs::PoseStampedConstPtr& msg)
//{
//    tf::poseMsgToEigen(msg->pose, state);
//    valid_state = true;
//}

//bool RosClient::RosTask::is_valid() const
//{
//    return valid_state;
//}

//void RosClient::RosTask::set_async_mode(bool async)
//{
//    async_mode = async;
//}


//const std::vector< std::string >& RosClient::getTaskList() const
//{
//    if(!_tasklist_srv.waitForExistence(ros::Duration(1.0)))
//    {
//        _tasklist_srv.getService();
//        throw std::runtime_error("Server for service " + _tasklist_srv.getService() + " is not running");
//    }

//    cartesian_interface::GetTaskList srv;

//    if(!_tasklist_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to call service " + _tasklist_srv.getService());
//    }

//    _tasklist = srv.response.distal_links;

//    return _tasklist;
//}

bool RosClient::update(double time, double period)
{
    callAvailable();
    return true;
}

//Eigen::Affine3d RosClient::RosTask::get_state() const
//{
//    return state;
//}

//void RosClient::RosTask::get_properties(std::string& base_link,
//                                      ControlType& ctrl_type,
//                                      TaskInterface& ifc_type
//                                     )
//{
//    cartesian_interface::GetTaskInfo srv;

//    if(async_mode)
//    {
//        dynamic_cast<ros::CallbackQueue *>(nh.getCallbackQueue())->callAvailable();
//        srv.response.base_link = cached_task_prop.base_link;
//        srv.response.task_state = cached_task_prop.task_state;
//        srv.response.control_mode = cached_task_prop.control_mode;
//        srv.response.task_interface = cached_task_prop.task_interface;
//        srv.response.distal_link = cached_task_prop.distal_link;
//    }
//    else if(!get_prop_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to get properties for task " + distal_link);
//    }

//    base_link = srv.response.base_link;
//    ctrl_type = ControlTypeFromString(srv.response.control_mode);
//    ifc_type = TaskInterfaceFromString(srv.response.task_interface);

//}

//void RosClient::RosTask::set_base_link(const std::string& base_link)
//{
//    cartesian_interface::SetTaskInfo srv;
//    srv.request.base_link = base_link;
//    if(!set_prop_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to set properties for task " + distal_link);
//    }
//    if(!srv.response.success)
//    {
//        throw std::runtime_error("Service " + set_prop_srv.getService() +
//            " returned an error: \n\t" + srv.response.message);
//    }

//    ROS_INFO("%s", srv.response.message.c_str());
//}

//void RosClient::RosTask::set_ctrl_mode(ControlType ctrl_type)
//{
//    cartesian_interface::SetTaskInfo srv;
//    srv.request.control_mode = ControlTypeAsString(ctrl_type);
//    if(!set_prop_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to set properties for task " + distal_link);
//    }
//    if(!srv.response.success)
//    {
//        throw std::runtime_error("Service " + set_prop_srv.getService() +
//            " returned an error: \n\t" + srv.response.message);
//    }

//    ROS_INFO("%s", srv.response.message.c_str());
//}


//void RosClient::RosTask::send_ref(const Eigen::Affine3d& ref)
//{
//    geometry_msgs::PoseStamped msg;
//    msg.header.stamp = ros::Time::now();
//    tf::poseEigenToMsg(ref, msg.pose);
//    ref_pub.publish(msg);
//}

//void RosClient::RosTask::send_vref(const Eigen::Vector6d& vref,
//                                 const std::string& base_frame)
//{
//    geometry_msgs::TwistStamped msg;
//    msg.header.frame_id = base_frame;
//    msg.header.stamp = ros::Time::now();
//    tf::twistEigenToMsg(vref, msg.twist);
//    vref_pub.publish(msg);
//}

//void RosClient::RosTask::send_waypoints(const Trajectory::WayPointVector& wp, bool incr)
//{
//    cartesian_interface::ReachPoseGoal goal;
//    goal.incremental = incr;

//    for(auto f : wp)
//    {
//        geometry_msgs::Pose frame;
//        tf::poseEigenToMsg(f.frame, frame);

//        goal.frames.push_back(frame);
//        goal.time.push_back(f.time);
//    }

//    if(!reach_action.waitForServer(ros::Duration(2.0)))
//    {
//        throw std::runtime_error("Unable to connect to action server for task " + distal_link);
//    }

//    reach_action.sendGoal(goal);

//}

//void XBot::Cartesian::RosClient::RosTask::abort()
//{
//    reach_action.cancelAllGoals();
//}


//bool RosClient::RosTask::wait_for_result(ros::Duration timeout)
//{
//    return reach_action.waitForResult(timeout);
//}

//bool RosClient::waitReachCompleted(const std::string& ee_name, double timeout_sec)
//{
//    auto task = get_task(ee_name, false);

//    return task->wait_for_result(ros::Duration(timeout_sec));
//}


//bool RosClient::getPoseReference(const std::string& end_effector,
//                                                Eigen::Affine3d& base_T_ref,
//                                                Eigen::Vector6d* base_vel_ref,
//                                                Eigen::Vector6d* base_acc_ref) const
//{
//    auto task = get_task(end_effector, false);

//    if(!task)
//    {
//        return false;
//    }

//    base_T_ref = task->get_state();

//    if(base_vel_ref) base_vel_ref->setZero();
//    if(base_acc_ref) base_acc_ref->setZero();

//    return true;
//}

//bool RosClient::getPoseReferenceRaw(const std::string& end_effector,
//                                                   Eigen::Affine3d& base_T_ref,
//                                                   Eigen::Vector6d* base_vel_ref,
//                                                   Eigen::Vector6d* base_acc_ref) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::setPoseReferenceRaw(const std::string& end_effector,
//                                                   const Eigen::Affine3d& base_T_ref)
//{
//    THROW_NOT_IMPL
//}

void RosClient::loadController(const std::string& controller_name,
                             const std::string& problem_description_name,
                             const std::string& problem_description_string,
                             const bool force_reload)
{
    cartesian_interface::LoadController srv;
    srv.request.controller_name = controller_name;
    srv.request.force_reload = force_reload;
    srv.request.problem_description_name = problem_description_name;
    srv.request.problem_description_string = problem_description_string;

    if(!_load_ctrl_srv.call(srv))
    {
        throw std::runtime_error("Unable to connect to " + _load_ctrl_srv.getService());
    }

    if(!srv.response.success)
    {
        throw std::runtime_error(_load_ctrl_srv.getService() + " responded with an error:\n\t" + srv.response.message);
    }

    ROS_INFO("%s", srv.response.message.c_str());

    std::string ns = nh().getNamespace();
    this->~RosClient();
    new(this) RosClient(ns);


}

bool RosClient::waitReachCompleted(const std::string & ee_name, double timeout_sec)
{
    auto cart_ros = std::dynamic_pointer_cast<ClientApi::CartesianRos>(getTask(ee_name));

    if(!cart_ros) return false;

    return cart_ros->waitReachCompleted(timeout_sec);
}


//RosClient::~RosClient()
//{

//}

//void RosClient::set_async_mode(bool async)
//{
//    for(auto pair: _task_map)
//    {
//        pair.second->set_async_mode(async);
//    }
//}



//const std::string& RosClient::getBaseLink(const std::string& ee_name) const
//{
//    auto task = get_task(ee_name, false);

//    ControlType ctrl_tmp;
//    TaskInterface ifc_tmp;
//    static std::string base_link_static; // NOTE: user must copy the output
//    task->get_properties(base_link_static, ctrl_tmp, ifc_tmp);

//    return base_link_static;

//}

//ControlType RosClient::getControlMode(const std::string& ee_name) const
//{
//    auto task = get_task(ee_name, false);

//    ControlType ctrl_tmp;
//    std::string base_link_tmp;
//    TaskInterface ifc_tmp;
//    task->get_properties(base_link_tmp, ctrl_tmp, ifc_tmp);

//    return ctrl_tmp;
//}

//bool RosClient::getCurrentPose(const std::string& end_effector, Eigen::Affine3d& base_T_ee) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::getPoseTarget(const std::string& end_effector, Eigen::Affine3d& base_T_ref) const
//{
//    THROW_NOT_IMPL
//}

//int RosClient::getCurrentSegmentId(const std::string & end_effector) const
//{
//    THROW_NOT_IMPL
//}

//State RosClient::getTaskState(const std::string& end_effector) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::reset()
//{
//    auto client = _nh.serviceClient<std_srvs::Trigger>("reset");

//    std_srvs::Trigger msg;
//    if(!client.call(msg))
//    {
//        throw std::runtime_error("Unable to reset");
//    }

//    ROS_INFO("%s", msg.response.message.c_str());

//    return true;
//}

//bool RosClient::setComPositionReference(const Eigen::Vector3d& base_com_ref)
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::setComVelocityReference(const Eigen::Vector3d& base_vel_ref)
//{
//    THROW_NOT_IMPL
//}


//bool RosClient::setControlMode(const std::string& ee_name, ControlType ctrl_type)
//{
//    auto task = get_task(ee_name, false);

//    task->set_ctrl_mode(ctrl_type);

//    return true;
//}

//bool RosClient::setPoseReference(const std::string& end_effector,
//                               const Eigen::Affine3d& base_T_ref)
//{
//    auto task = get_task(end_effector, false);

//    task->send_ref(base_T_ref);

//    return true;
//}

//bool RosClient::setVelocityReferenceAsync(const std::string& ee_name,
//                                        const Eigen::Vector6d& vref,
//                                        double timeout_sec)
//{
//    auto task = get_task(ee_name, false);

//    task->send_vref_async(vref, timeout_sec);

//    return true;
//}

//bool RosClient::stopVelocityReferenceAsync(const std::string& ee_name)
//{
//    auto task = get_task(ee_name, false);

//    task->stop_vref_async();

//    return true;
//}


//bool RosClient::setVelocityReference(const std::string& end_effector,
//                                   const Eigen::Vector6d& base_vel_ref)
//{
//    return setVelocityReference(end_effector, base_vel_ref);
//}

//bool RosClient::setVelocityReference(const std::string & end_effector,
//                                   const Eigen::Vector6d & base_vel_ref,
//                                   const std::string & base_frame)
//{
//    auto task = get_task(end_effector, false);

//    task->send_vref(base_vel_ref, base_frame);

//    return true;
//}

//bool RosClient::setTargetComPosition(const Eigen::Vector3d& base_com_ref, double time)
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::setTargetPose(const std::string& end_effector, const Eigen::Affine3d& base_T_ref, double time)
//{
//    return setTargetPose(end_effector, base_T_ref, time, false);
//}

//bool RosClient::setTargetPose(const std::string& end_effector,
//                            const Eigen::Affine3d& base_T_ref,
//                            double time,
//                            bool incremental)
//{
//    Trajectory::WayPointVector wpv;
//    wpv.emplace_back(base_T_ref, time);
//    return setWayPoints(end_effector, wpv, incremental);
//}

//bool RosClient::setWayPoints(const std::string& end_effector,
//                           const Trajectory::WayPointVector& way_points,
//                           bool incremental)
//{
//    auto task = get_task(end_effector, false);

//    task->send_waypoints(way_points, incremental);

//    return true;
//}

//bool RosClient::abort(const std::string& end_effector)
//{
//    auto task = get_task(end_effector, false);

//    task->abort();

//    return true;
//}

//void RosClient::getAccelerationLimits(const std::string& ee_name, double& max_acc_lin, double& max_acc_ang) const
//{
//    get_task(ee_name, false);

//    if(!_nh.getParam(ee_name + "/max_acceleration_linear", max_acc_lin) ||
//       !_nh.getParam(ee_name + "/max_acceleration_angular", max_acc_ang))
//    {
//        throw std::runtime_error("Undefine acceleration limits for task " + ee_name);
//    }
//}

//bool RosClient::getComPositionReference(Eigen::Vector3d& w_com_ref, Eigen::Vector3d* base_vel_ref, Eigen::Vector3d* base_acc_ref) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::getReferencePosture(Eigen::VectorXd& qref) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::getReferencePosture(XBot::JointNameMap& qref) const
//{
//    THROW_NOT_IMPL
//}

//void RosClient::getVelocityLimits(const std::string& ee_name, double& max_vel_lin, double& max_vel_ang) const
//{
//    get_task(ee_name, false);

//    if(!_nh.getParam(ee_name + "/max_velocity_linear", max_vel_lin) ||
//       !_nh.getParam(ee_name + "/max_velocity_angular", max_vel_ang))
//    {
//        throw std::runtime_error("Undefine velocity limits for task " + ee_name);
//    }
//}

namespace
{
    bool call_reset_world_service(ros::NodeHandle& nh,
                                  const Eigen::Affine3d& w_T_new_world,
                                  const std::string& ee_name)
    {
        auto client = nh.serviceClient<cartesian_interface::ResetWorld>("reset_world");
        if(!client.waitForExistence(ros::Duration(3.0)))
        {
            throw std::runtime_error("unable to reset world, service unavailable");
        }

        cartesian_interface::ResetWorld srv;
        tf::poseEigenToMsg(w_T_new_world, srv.request.new_world);
        srv.request.from_link = ee_name;

        if(!client.call(srv))
        {
            throw std::runtime_error("unable to reset world, service call failed");
        }

        ROS_INFO("%s", srv.response.message.c_str());

        if(!srv.response.success)
        {
            throw std::runtime_error("unable to reset world, service responded with an error");
        }

        return true;
    }
}

bool RosClient::resetWorld(const Eigen::Affine3d& w_T_new_world)
{
    return ::call_reset_world_service(nh(), w_T_new_world, "");
}

bool XBot::Cartesian::RosClient::resetWorld(const std::string& ee_name)
{
    return ::call_reset_world_service(nh(), Eigen::Affine3d::Identity(), ee_name);
}


//void RosClient::setAccelerationLimits(const std::string& ee_name, double max_acc_lin, double max_acc_ang)
//{
//    double dummy;
//    getAccelerationLimits(ee_name, dummy, dummy);

//    _nh.setParam(ee_name + "/max_acceleration_linear",  max_acc_lin);
//    _nh.setParam(ee_name + "/max_acceleration_angular", max_acc_ang);

//    std_srvs::Trigger srv;

//    if(!_upd_limits_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to connect to " + _upd_limits_srv.getService());
//    }

//    if(!srv.response.success)
//    {
//        throw std::runtime_error(_upd_limits_srv.getService() + " responded with an error:\n\t" + srv.response.message);
//    }

//    ROS_INFO("%s", srv.response.message.c_str());
//}

//void RosClient::setVelocityLimits(const std::string& ee_name, double max_vel_lin, double max_vel_ang)
//{
//    double dummy;
//    getVelocityLimits(ee_name, dummy, dummy);

//    _nh.setParam(ee_name + "/max_velocity_linear",  max_vel_lin);
//    _nh.setParam(ee_name + "/max_velocity_angular", max_vel_ang);

//    std_srvs::Trigger srv;

//    if(!_upd_limits_srv.call(srv))
//    {
//        throw std::runtime_error("Unable to connect to " + _upd_limits_srv.getService());
//    }

//    if(!srv.response.success)
//    {
//        throw std::runtime_error(_upd_limits_srv.getService() + " responded with an error:\n\t" + srv.response.message);
//    }

//    ROS_INFO("%s", srv.response.message.c_str());
//}

//bool RosClient::reset(double time)
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::getTargetComPosition(Eigen::Vector3d& w_com_ref) const
//{
//    THROW_NOT_IMPL
//}

//bool RosClient::setWayPoints(const std::string& end_effector, const Trajectory::WayPointVector& way_points)
//{
//    return setWayPoints(end_effector, way_points, false);
//}

//bool RosClient::setReferencePosture(const XBot::JointNameMap& qref)
//{
//    sensor_msgs::JointState msg;
//    msg.header.stamp = ros::Time::now();

//    for(auto p : qref)
//    {
//        msg.name.push_back(p.first);
//        msg.position.push_back(p.second);
//    }

//    _posture_pub.publish(msg);

//    return true;
//}

//bool RosClient::setBaseLink(const std::string& ee_name, const std::string& new_base_link)
//{
//    auto task = get_task(ee_name, false);

//    task->set_base_link(new_base_link);

//    return true;
//}




//RosClient::RosInitHelper::RosInitHelper(std::string node_namespace)
//{
//    if(!ros::ok())
//    {
//        std::string ns_arg = "__ns:=";
//        ns_arg += node_namespace;
//        std::vector<const char *> args {"", ns_arg.c_str()};

//        int argc = args.size();

//        ros::init(argc, (char **)args.data(), "cartesio_ros",
//                    ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
//        ROS_WARN("Initializing roscpp under namespace %s with anonymous name %s",
//                    ros::this_node::getNamespace().c_str(),
//                    ros::this_node::getName().c_str()
//                );
//    }
//}


//RosClient::VelocityPublisherAsync::VelocityPublisherAsync(std::string topic_name,
//                                                        ros::Duration period):
//    _spinner(1, &_queue),
//    _timeout(0)
//{
//    ros::NodeHandle nh("cartesian");
//    nh.setCallbackQueue(&_queue);

//    _vref_pub = nh.advertise<geometry_msgs::TwistStamped>(topic_name, 1);

//    _timer = nh.createTimer(period, &VelocityPublisherAsync::timer_callback, this);

//    tf::twistEigenToMsg(Eigen::Vector6d::Zero(), _twist.twist);

//}

//void RosClient::VelocityPublisherAsync::set_vref(const Eigen::Vector6d& vref)
//{
//    std::lock_guard<std::recursive_mutex> lock(_mutex);

//    tf::twistEigenToMsg(vref, _twist.twist);
//}

//void RosClient::VelocityPublisherAsync::start(ros::Duration timeout)
//{
//    std::lock_guard<std::recursive_mutex> lock(_mutex);

//    if(timeout < ros::Duration(0))
//    {
//        _timeout = ros::Time(0);
//    }
//    else
//    {
//        _timeout = ros::Time::now() + timeout;
//    }

//    _spinner.start();
//    _timer.start();
//}

//void RosClient::VelocityPublisherAsync::stop()
//{
//    std::lock_guard<std::recursive_mutex> lock(_mutex);

//    _timer.stop();
//    _spinner.stop();

//    tf::twistEigenToMsg(Eigen::Vector6d::Zero(), _twist.twist);
//}

//void RosClient::VelocityPublisherAsync::timer_callback(const ros::TimerEvent& ev)
//{
//   std::lock_guard<std::recursive_mutex> lock(_mutex);

//   auto now = ros::Time::now();

//    if(_timeout != ros::Time(0) && now >= _timeout)
//    {
//        _timer.stop();
//        tf::twistEigenToMsg(Eigen::Vector6d::Zero(), _twist.twist);
//        return;
//    }

//    _twist.header.stamp = now;
//    _vref_pub.publish(_twist);
//}

//void RosClient::RosTask::send_vref_async(const Eigen::Vector6d& vref, double timeout_duration)
//{
//    vref_async.set_vref(vref);
//    vref_async.start(ros::Duration(timeout_duration));
//}

//void RosClient::RosTask::stop_vref_async()
//{
//    vref_async.stop();
//}

//TaskInterface RosClient::getTaskInterface(const std::string& end_effector) const
//{
//    auto task = get_task(end_effector, false);

//    ControlType ctrl_tmp;
//    TaskInterface ifc_tmp;
//    std::string base_link_tmp; // NOTE: user must copy the output
//    task->get_properties(base_link_tmp, ctrl_tmp, ifc_tmp);

//    return ifc_tmp;
//}

//bool RosClient::getDesiredInteraction(const std::string& end_effector,
//                                                     Eigen::Vector6d& force,
//                                                     Eigen::Matrix6d& stiffness,
//                                                     Eigen::Matrix6d& damping) const
//{
//    auto itask = get_interaction_task(end_effector, false);

//    force.setZero();
//    itask->get_impedance(stiffness, damping);

//    return true;
//}

//bool RosClient::setDesiredDamping(const std::string& end_effector,
//                                                 const Eigen::Matrix6d& d)
//{
//    auto itask = get_interaction_task(end_effector, false);
//    Eigen::Matrix6d ktmp, dtmp;
//    itask->get_impedance(ktmp, dtmp);
//    itask->send_impedance(ktmp, d);

//    return true;
//}

//bool RosClient::setDesiredStiffness(const std::string& end_effector,
//                                                   const Eigen::Matrix6d& k)
//{
//    auto itask = get_interaction_task(end_effector, false);
//    Eigen::Matrix6d ktmp, dtmp;
//    itask->get_impedance(ktmp, dtmp);
//    itask->send_impedance(k, dtmp);

//    return true;
//}

//bool RosClient::setForceReference(const std::string& end_effector,
//                                                 const Eigen::Vector6d& force)
//{
//    auto itask = get_interaction_task(end_effector, false);

//    itask->send_force(force);

//    return true;
//}

//RosClient::RosInteractionTask::RosInteractionTask(ros::NodeHandle nh,
//                                                std::string distal_link):

//    RosTask(nh, distal_link)
//{
//    k.setZero();
//    d.setZero();

//    impedance_pub = nh.advertise<cartesian_interface::Impedance6>(distal_link + "/impedance", 1);
//    force_pub = nh.advertise<geometry_msgs::WrenchStamped>(distal_link + "/force", 1);
//    get_imp_srv = nh.serviceClient<cartesian_interface::GetImpedance>(distal_link + "/get_impedance");

//    get_imp_srv.waitForExistence(ros::Duration(1.0));
//}

//void XBot::Cartesian::RosClient::RosInteractionTask::get_impedance(Eigen::Matrix6d& k,
//                                                                 Eigen::Matrix6d& d) const
//{
//    cartesian_interface::GetImpedanceRequest req;
//    cartesian_interface::GetImpedanceResponse res;

//    if(!get_imp_srv.call(req, res))
//    {
//        throw std::runtime_error("Unable to get impedance for interaction task " + get_distal_link());
//    }

//    k.setZero();
//    d.setZero();
//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = i; j < 3; j++)
//        {
//            k(i,j) = k(j,i) = res.linear.stiffness[i+3*j];
//            d(i,j) = d(j,i) = res.linear.damping[i+3*j];

//            k(i+3,j+3) = k(j+3,i+3) = res.angular.stiffness[i+3*j];
//            d(i+3,j+3) = d(j+3,i+3) = res.angular.damping[i+3*j];
//        }
//    }
//}

//void XBot::Cartesian::RosClient::RosInteractionTask::send_force(const Eigen::Vector6d& f)
//{
//    geometry_msgs::WrenchStamped msg;
//    msg.header.stamp = ros::Time::now();

//    tf::wrenchEigenToMsg(f, msg.wrench);

//    force_pub.publish(msg);

//}

//void XBot::Cartesian::RosClient::RosInteractionTask::send_impedance(const Eigen::Matrix6d& k,
//                                                                  const Eigen::Matrix6d& d)
//{
//    cartesian_interface::Impedance6 msg;
//    msg.header.stamp = ros::Time::now();

//    for(int i = 0; i < 3; i++)
//    {
//        for(int j = 0; j < 3; j++)
//        {
//            msg.linear.stiffness[i+3*j] = k(i,j);
//            msg.linear.damping[i+3*j] = d(i,j);

//            msg.angular.stiffness[i+3*j] =  k(i+3,j+3);
//            msg.angular.damping[i+3*j] = d(i+3,j+3);
//        }
//    }

//    impedance_pub.publish(msg);
//}

//const std::string & XBot::Cartesian::RosClient::RosTask::get_distal_link() const
//{
//    return distal_link;
//}






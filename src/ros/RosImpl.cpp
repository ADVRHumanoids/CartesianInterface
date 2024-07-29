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
#include "ros/client_api/InteractionRos.h"

#include <xbot2_interface/logger.h>

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
                                                  srv_info.response.lib_name,
                                                  nh);

        tasks.push_back(t);

    }

    ProblemDescription ik_pb(tasks);

    attempts = 100;
    while(--attempts && !ik_pb.validate())
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
    CartesianInterfaceImpl(::construct_problem(nh()))
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

bool RosClient::update(double time, double period)
{
    callAvailable();
    return true;
}

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

bool RosClient::setStiffnessTransition(const std::string& end_effector,
                      const Interpolator<Eigen::Matrix6d>::WayPointVector & way_points)
{
    auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(end_effector));

    if(!interaction_ros) return false;

    return interaction_ros->setStiffnessTransition(way_points);
}

bool RosClient::waitStiffnessTransitionCompleted(const std::string& ee_name, double timeout_sec)
{
    auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(ee_name));

    if(!interaction_ros) return false;

    return interaction_ros->waitTransitionCompleted(timeout_sec);
}

bool RosClient::abortStiffnessTransition(const std::string& end_effector)
{
	auto interaction_ros = std::dynamic_pointer_cast<ClientApi::InteractionRos>(getTask(end_effector));

    if(!interaction_ros) return false;
	
	interaction_ros->abortStiffnessTransition();
    return true;
}

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





#ifndef CARTESIANROSAPI_H
#define CARTESIANROSAPI_H

#include "TaskRos.h"

#include <cartesian_interface/problem/Cartesian.h>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <cartesian_interface/ReachPoseAction.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <cartesian_interface/SetTaskInfo.h>


namespace XBot { namespace Cartesian {

class ReachActionManager
{

public:

    ReachActionManager(ros::NodeHandle nh,
                       std::string task_name,
                       CartesianTask::Ptr task);

    void run();

private:

    typedef actionlib::SimpleActionServer<cartesian_interface::ReachPoseAction> ActionServer;
    typedef std::shared_ptr<ActionServer> ActionServerPtr;

    enum class ReachActionState { IDLE, ACCEPTED, RUNNING, COMPLETED };

    void run_state_idle();
    void run_state_accepted();
    void run_state_running();
    void run_state_completed();

    ActionServerPtr _action_server;
    CartesianTask::Ptr _task;
    ReachActionState _state;
    std::string _name;

};

class CartesianRos : public TaskRos
{

public:

    CartesianRos(RosContext ctx,
                 TaskDescription::Ptr task);

    virtual void run(ros::Time time) override;

protected:

private:

    void publish_ref(ros::Time time);

    void online_position_reference_cb(geometry_msgs::PoseStampedConstPtr  msg);
    void online_velocity_reference_cb(geometry_msgs::TwistStampedConstPtr msg);
    bool get_task_info_cb(cartesian_interface::GetTaskInfoRequest& req,
                        cartesian_interface::GetTaskInfoResponse& res);
    bool set_task_info_cb(cartesian_interface::SetTaskInfoRequest& req,
                        cartesian_interface::SetTaskInfoResponse& res);


    ros::Publisher _pose_ref_pub, _vel_ref_pub;
    ros::Subscriber _pose_ref_sub, _vel_ref_sub;
    ros::ServiceServer _get_info_srv, _set_info_srv;

    CartesianTask::Ptr _cart;

    std::unique_ptr<ReachActionManager> _reach_action_manager;



};

} }

#endif // CARTESIANROSAPI_H

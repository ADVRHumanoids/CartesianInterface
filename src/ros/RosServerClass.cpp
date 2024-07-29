#include <cartesian_interface/ros/RosServerClass.h>
#include "utils/RosUtils.h"

using namespace std::chrono_literals;
using namespace XBot::Cartesian;

namespace  {

geometry_msgs::msg::Pose get_normalized_pose(const geometry_msgs::msg::Pose& pose)
{
    Eigen::Vector4d coeff(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

    if(coeff.squaredNorm() == 0)
    {
        coeff[0] = 1.0;
    }

    coeff /= coeff.norm();

    geometry_msgs::msg::Pose norm_pose = pose;
    norm_pose.orientation.w = coeff[0];
    norm_pose.orientation.x = coeff[1];
    norm_pose.orientation.y = coeff[2];
    norm_pose.orientation.z = coeff[3];

    return norm_pose;

}

}

RosServerClass::Options::Options():
    tf_prefix("ci"),
    ros_namespace("cartesian"),
    publish_tf(true)
{

}


RosServerClass::RosServerClass(CartesianInterfaceImpl::Ptr intfc,
                               Options opt,
                               rclcpp::Node::SharedPtr node):
    _ci(intfc),
    _model(intfc->getModel()),
    _opt(opt),
    _spin_node(false)
{
    // create ros2 node if not provided
    if(node)
    {
        _node = node;
    }
    else
    {
        _node = rclcpp::Node::make_shared("cartesio_ros2_server");
        _node = _node->create_sub_node(opt.ros_namespace);
        _spin_node = true;
    }

    // ros context
    _ros_ctx = std::make_shared<RosContext>(_node, _tf_prefix, intfc->getContext());
    _tf_prefix = _opt.tf_prefix;
    _tf_prefix_slash = _ros_ctx->tf_prefix_slash();

    // init all
    init_state_broadcasting();
    init_rspub();
    init_task_list_service();
    init_reset_world_service();
    init_heartbeat_pub();
    init_load_ros_task_api();

}

void RosServerClass::init_state_broadcasting()
{
    _solution_pub = _node->create_publisher<JointState>("solution", 1);
}


void RosServerClass::init_rspub()
{
    _rspub = std::make_unique<Utils::RobotStatePublisher>(_node, _model);

    _com_pub = _node->create_publisher<PointStamped>("com_position", 1);
}

void RosServerClass::publish_ref_tf(rclcpp::Time time)
{
    /* Publish TF */
    _rspub->publishTransforms(time, _tf_prefix);
    
    /* Publish CoM position */
    Eigen::Vector3d com;
    _model->getCOM(com);
    
    PointStamped com_msg;
    com_msg.point = tf2::toMsg(com);
    com_msg.header.frame_id = _tf_prefix_slash + "world";
    com_msg.header.stamp = time;
    _com_pub->publish(com_msg);
    
}

void RosServerClass::heartbeat_cb()
{
    _heartbeat_pub->publish(std_msgs::msg::Empty());
}

void XBot::Cartesian::RosServerClass::publish_solution(rclcpp::Time time)
{
    sensor_msgs::msg::JointState msg;
    std::vector<geometry_msgs::msg::WrenchStamped> w_msg;
    Eigen::VectorXd _sol_q, _sol_qdot, _sol_tau;
    auto solution = _ci->getSolution();

    if(!_wrench_pubs_inited)
    {
        for(auto& p : solution)
        {
            if(p.first.find("force_") != 0)
            {
                continue;
            }
            _wrench_pubs.push_back(_node->create_publisher<WrenchStamped>(p.first, 1));
        }
        _wrench_pubs_inited = true;
    }

    bool no_subsribers = true;

    if(_solution_pub->get_subscription_count() != 0)
    {
        no_subsribers = false;
    }

    for(auto& pub : _wrench_pubs)
    {
        if(pub->get_subscription_count() != 0)
        {
            no_subsribers = false;
        }
    }

    if(no_subsribers)
        return;
    
    _model->getJointPosition(_sol_q);
    _model->getJointVelocity(_sol_qdot);
    _model->getJointEffort(_sol_tau);

    // get minimal representation of q
    // this has size = nv

    // TODO use positionToMinimal instead of difference
    _sol_q = _model->positionToMinimal(_sol_q);


    for(auto& p : solution)
    {
        if(p.first.find("force_") != 0)
        {
            continue;
        }

        WrenchStamped w;
        w.header.stamp = time;
        auto frame = p.first.substr(6); // removes "force_"

        Eigen::Affine3d w_T_f = _model->getPose(frame);
        Eigen::Vector6d ww;
        ww.setZero();
        if(p.second.size() == 3)
            ww.segment(0,3) = p.second;
        else
            ww = p.second;

        //in local frame
        ww.segment(0,3) = w_T_f.linear().inverse() * ww.segment(0,3);
        ww.segment(3,3) = w_T_f.linear().inverse() * ww.segment(3,3);

        w.wrench.force.x = ww[0];
        w.wrench.force.y = ww[1];
        w.wrench.force.z = ww[2];
        w.wrench.torque.x = ww[3];
        w.wrench.torque.y = ww[4];
        w.wrench.torque.z = ww[5];

        w.header.frame_id = "ci/" + frame;

        w_msg.push_back(w);
    }


    for(unsigned int i = 0; i < _wrench_pubs.size(); ++i)
        _wrench_pubs[i]->publish(w_msg[i]);

    msg.header.stamp = time;
    msg.name.reserve(_model->getNv());
    msg.position.reserve(_model->getNv());
    msg.velocity.reserve(_model->getNv());
    int i = 0;
    for(const std::string& jname : _model->getVNames())
    {
        msg.name.push_back(jname);
        msg.position.push_back(_sol_q[i]);
        msg.velocity.push_back(_sol_qdot[i]);
        msg.effort.push_back(_sol_tau[i]);
        i++;
    }
    
    _solution_pub->publish(msg);
}


void RosServerClass::run()
{
    if(_spin_node)
    {
        rclcpp::spin_some(_node);
    }
    
    auto now = _node->get_clock()->now();

    if(_opt.publish_tf)
    {
        publish_ref_tf(now);
    }

    publish_solution(now);

    for(auto t : _ros_tasks)
    {
        t->run(now);
    }
}

void XBot::Cartesian::RosServerClass::init_task_list_service()
{
    std::string srv_name = "get_task_list";

    _tasklist_srv = ::create_service<GetTaskList>(_node,
                                                  srv_name,
                                                  &RosServerClass::task_list_cb,
                                                  this);
}

bool XBot::Cartesian::RosServerClass::task_list_cb(GetTaskList::Request::ConstSharedPtr req,
                                                   GetTaskList::Response::SharedPtr res)
{
    
    for(const auto& tname : _ci->getTaskList())
    {
        auto t = _ci->getTask(tname);

        res->names.push_back(t->getName());
        res->types.push_back(t->getType());
    }
    
    return true;
}

RosServerClass::~RosServerClass()
{
}

XBot::ModelInterface::ConstPtr RosServerClass::getModel() const
{
    return _model;
}


void XBot::Cartesian::RosServerClass::init_reset_world_service()
{
    _reset_world_srv = ::create_service<ResetWorld>(_node,
                                                    "reset_world",
                                                    &RosServerClass::reset_world_cb,
                                                    this);

    _reset_base_srv = ::create_service<SetTransform>(_node,
                                                     "reset_base",
                                                     &RosServerClass::reset_base_cb,
                                                     this);
}

void RosServerClass::init_heartbeat_pub()
{
    _heartbeat_pub = _node->create_publisher<Empty>("heartbeat", 1);

    _heartbeat_timer = _node->create_timer(100ms,
                                           [this](){
                                               heartbeat_cb();
                                           });
}

void RosServerClass::init_load_ros_task_api()
{
    for(auto tname : _ci->getTaskList())
    {
        auto t = _ci->getTask(tname);

        auto task_ros = ServerApi::TaskRos::MakeInstance(t, _ros_ctx);

        _ros_tasks.push_back(task_ros);
    }
}

bool XBot::Cartesian::RosServerClass::reset_world_cb(ResetWorld::Request::ConstSharedPtr req,
                                                     ResetWorld::Response::SharedPtr res)
{
    Eigen::Affine3d new_world;

    if(req->from_link == "")
    {
        tf2::fromMsg(get_normalized_pose(req->new_world), new_world);
    }
    else
    {
        if(!_model->getPose(req->from_link, new_world))
        {
            res->success = false;
            res->message = "Link " + req->from_link + " undefined. World could not be changed.";
            return true;
        }
    }

    if(_ci->resetWorld(new_world))
    {
        res->success = true;
        res->message = "World was changed successfully.";
    }
    else
    {
        res->success = false;
        res->message = "World could not be changed.";
    }

    return true;
}

bool RosServerClass::reset_base_cb(SetTransform::Request::ConstSharedPtr req,
                                   SetTransform::Response::SharedPtr res)
{
    Eigen::Affine3d nw_T_base, w_T_nw, w_T_base;
    tf2::fromMsg(get_normalized_pose(req->pose), nw_T_base);

    if(!_model->getFloatingBasePose(w_T_base))
    {
        res->success = false;
        res->message = "model is not floating base";
        return true;
    }

    w_T_nw = w_T_base * nw_T_base.inverse();

    _ci->resetWorld(w_T_nw);

    res->success = true;
    res->message = "successfully set floating base pose to model";
    return true;
}

#include <cartesian_interface/markers/CartesianMarker.h>
#include <cartesian_interface/srv/get_task_info.hpp>
#include <numeric>
#include <xbot2_interface/logger.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

#include "ros/client_api/CartesianRos.h"
#include "utils/RosUtils.h"

#define SECS 5

using namespace XBot::Cartesian;
using namespace std::chrono_literals;

CartesianMarker::CartesianMarker(const std::string& task_name,
                                 const urdf::Model &robot_urdf,
                                 const unsigned int control_type,
                                 std::string tf_prefix,
                                 const bool use_mesh
                                 ):
    CartesianMarker(task_name,
                    robot_urdf,
                    control_type,
                    rclcpp::Node::make_shared("cartesian_marker")->create_sub_node("cartesian"),
                    tf_prefix,
                    use_mesh)
{

}

CartesianMarker::CartesianMarker(const std::string& task_name,
                                 const urdf::Model& robot_urdf,
                                 const unsigned int control_type,
                                 rclcpp::Node::SharedPtr node,
                                 std::string tf_prefix,
                                 const bool use_mesh):
    _urdf(robot_urdf),
    _server(task_name + "_cartesian_marker_server", node),
    _tf_prefix(tf_prefix),
    _menu_entry_counter(0),
    _control_type(1),_is_continuous(1), _task_active(-1), _position_feedback_active(-1),
    _node(node),
    _use_mesh(use_mesh)
{
    _task = std::make_shared<ClientApi::CartesianRos>(task_name, node);

    _distal_link = _task->getDistalLink();

    _base_link = _task->getBaseLink();

    _urdf.getLinks(_links);

    _start_pose = getRobotActualPose();

    _actual_pose = _start_pose;

    MakeMarker(_distal_link, _base_link, false, control_type, true);

    MakeMenu();

    _server.applyChanges();


    InteractiveMarkerFeedback::SharedPtr foo;
    resetMarker(foo);

    _clear_service = ::create_service<EmptySrv>(_node,
                                                _int_marker.name + "/clear_marker",
                                                &CartesianMarker::clearMarker,
                                                this);

    _spawn_service = ::create_service<EmptySrv>(_node,
                                                _int_marker.name + "/spawn_marker",
                                                &CartesianMarker::spawnMarker,
                                                this);


    std::string topic_name = task_name + "/wp";
    _way_points_pub = _node->create_publisher<PoseArray>(topic_name, 1);

}


CartesianMarker::~CartesianMarker()
{

}

void CartesianMarker::MakeMenu()
{
    namespace pl = std::placeholders;

    _menu_entry_counter = 0;

    // reset marker
    _reset_marker_entry = _menu_handler.insert("Reset Marker",
                                               std::bind(std::mem_fn(&CartesianMarker::resetMarker),
                                                         this,
                                                         pl::_1));
    _menu_entry_counter++;


    // global control
    _global_control_entry
        = _menu_handler.insert("Global Ctrl",
                               std::bind(std::mem_fn(&CartesianMarker::setControlGlobalLocal),
                                         this,
                                         pl::_1));

    _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::UNCHECKED);

    _menu_entry_counter++;


    // continuous control
    _continuous_control_entry
        = _menu_handler.insert("Continuous Ctrl",
                               std::bind(std::mem_fn(&CartesianMarker::setContinuousCtrl),
                                         this,
                                         pl::_1));

    _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::UNCHECKED);

    _menu_entry_counter++;

    // add waypoint
    _way_point_entry = _menu_handler.insert("Add WayPoint");
    _menu_handler.setVisible(_way_point_entry, true);
    _menu_entry_counter++;
    _T_entry = _menu_handler.insert(_way_point_entry, "T [sec]");
    _menu_entry_counter++;
    offset_menu_entry = _menu_entry_counter;
    for ( int i = 0; i < SECS; i++ )
    {
        std::ostringstream s;
        s <<i+1;
        _T_last = _menu_handler.insert(_T_entry,
                                       s.str(),
                                       std::bind(std::mem_fn(&CartesianMarker::wayPointCallBack),
                                                 this,
                                                 pl::_1));
        _menu_entry_counter++;
        _menu_handler.setCheckState(_T_last, interactive_markers::MenuHandler::UNCHECKED );
    }
    _reset_all_way_points_entry
        = _menu_handler.insert(_way_point_entry,
                               "Reset All",
                               std::bind(std::mem_fn(&CartesianMarker::resetAllWayPoints),
                                         this,
                                         pl::_1));
    _menu_entry_counter++;
    _reset_last_way_point_entry
        = _menu_handler.insert(_way_point_entry,
                               "Reset Last",
                               std::bind(std::mem_fn(&CartesianMarker::resetLastWayPoints),
                                         this,
                                         pl::_1));
    _menu_entry_counter++;

    _send_way_points_entry = _menu_handler.insert(_way_point_entry,
                                                  "Send",
                                                  std::bind(std::mem_fn(
                                                                &CartesianMarker::sendWayPoints),
                                                            this,
                                                            pl::_1));
    _menu_entry_counter++;

    _properties_entry = _menu_handler.insert("Properties");
    _menu_entry_counter++;
    _task_is_active_entry
        = _menu_handler.insert(_properties_entry,
                               "Task Active",
                               std::bind(std::mem_fn(&CartesianMarker::activateTask), this, pl::_1));
    _menu_entry_counter++;
    _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED );
    _position_feedback_is_active_entry
        = _menu_handler.insert(_properties_entry,
                               "Position FeedBack Active",
                               std::bind(std::mem_fn(&CartesianMarker::activatePositionFeedBack),
                                         this,
                                         pl::_1));
    _menu_entry_counter++;
    _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::CHECKED );




    _base_link_entry = _menu_handler.insert(_properties_entry, "Base Link");
    _menu_entry_counter++;
    for(unsigned int i = 0; i < _links.size(); ++i)
    {
        if(_distal_link != _links.at(i)->name)
        {
            interactive_markers::MenuHandler::EntryHandle link_entry = _menu_handler.insert(_base_link_entry,
                                                                                            _links.at(i)->name, std::bind(std::mem_fn(&CartesianMarker::changeBaseLink), this, pl::_1));

            _menu_entry_counter++;

            if(_base_link.compare("world") == 0 && (_links.at(i)->name).compare("world") == 0){
                _menu_handler.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
                _base_link_entry_active = link_entry;}
            else if(_base_link.compare(_links.at(i)->name) == 0){
                _menu_handler.setCheckState(link_entry, interactive_markers::MenuHandler::CHECKED );
                _base_link_entry_active = link_entry;}
            else
                _menu_handler.setCheckState(link_entry, interactive_markers::MenuHandler::UNCHECKED );

            _link_entries.push_back(link_entry);

            _map_link_entry[_links.at(i)->name] = _menu_entry_counter;
        }
    }


    _menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
    _menu_control.always_visible = true;

    _int_marker.controls.push_back(_menu_control);

    _menu_handler.apply(_server, _int_marker.name);
}

void CartesianMarker::sendWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    if(_waypoints.empty()) return;

    std::partial_sum(_T.begin(), _T.end(), _T.begin());

    Trajectory::WayPointVector wpv;

    for(int i = 0; i < _waypoints.size(); i++)
    {
        Trajectory::WayPoint wp;

        tf2::fromMsg(_waypoints[i], wp.frame);
        wp.time = _T.at(i);

        wpv.push_back(wp);
    }

    _task->setWayPoints(wpv);
    publishWP(_waypoints);

    _waypoints.clear();
    _T.clear();
}

void CartesianMarker::resetLastWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    namespace pl = std::placeholders;

    _T.pop_back();

    _waypoints.pop_back();

    clearMarker(_req, _res);

    RCLCPP_INFO(_node->get_logger(), "RESET LAST WAYPOINT!");

    if(_waypoints.empty())
    {
        spawnMarker(_req, _res);
    }
    else
    {
        if(_server.empty())
        {
            tf2::fromMsg(_waypoints.back(), _start_pose);

            _actual_pose = _start_pose;

            _int_marker.pose = _waypoints.back();

            _server.insert(_int_marker,
                           std::bind(std::mem_fn(&CartesianMarker::MarkerFeedback), this, pl::_1));

            _menu_handler.apply(_server, _int_marker.name);

            _server.applyChanges();
        }
    }

    publishWP(_waypoints);
}

void CartesianMarker::resetAllWayPoints(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    _T.clear();
    _waypoints.clear();
    resetMarker(feedback);
    publishWP(_waypoints);
    RCLCPP_INFO(_node->get_logger(), "RESETTING ALL WAYPOINTS!");
}

void CartesianMarker::resetMarker(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    if(_task->getActivationState() == ActivationState::Disabled)
    {
        _activateTask(false);
    }
    else
    {
        _activateTask(true);

        if(_task->getControlMode() == ControlType::Position)
        {
            _activatePositionFeedBack(true);
        }
        else
        {
            _activatePositionFeedBack(false);
        }
    }

    auto current_base_link = _task->getBaseLink();
    if(current_base_link != _base_link)
    {
        auto feedback = std::make_shared<InteractiveMarkerFeedback>();
        feedback->menu_entry_id = _map_link_entry[current_base_link];
        changeBaseLink(feedback);
    }


    clearMarker(_req, _res);
    spawnMarker(_req, _res);
}

void CartesianMarker::publishWP(const std::vector<geometry_msgs::msg::Pose>& wps)
{
    PoseArray msg;

    for(unsigned int i = 0; i < wps.size(); ++i)
    {
        msg.poses.push_back(wps[i]);
    }

    msg.header.frame_id = _tf_prefix + _base_link;

    msg.header.stamp = _node->now();

    _way_points_pub->publish(msg);
}

void CartesianMarker::wayPointCallBack(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    tf2::fromMsg(feedback->pose, _actual_pose);

    Eigen::Quaterniond actual_q(_actual_pose.linear());

    double T = double(feedback->menu_entry_id-offset_menu_entry);

    if(_is_continuous == 1)
    {
        RCLCPP_INFO(_node->get_logger(),
                    "\n %s set waypoint @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n "
                    "of %.1f secs",
                    _int_marker.name.c_str(),
                    _actual_pose.translation().x(),
                    _actual_pose.translation().y(),
                    _actual_pose.translation().z(),
                    actual_q.x(),
                    actual_q.y(),
                    actual_q.z(),
                    actual_q.w(),
                    T);

        _waypoints.push_back(feedback->pose);
        _T.push_back(T);

        publishWP(_waypoints);

    }
}

void CartesianMarker::changeBaseLink(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    int entry_id = feedback->menu_entry_id;

    std::string new_base_link;
    _menu_handler.getTitle(entry_id, new_base_link);

    if(new_base_link.compare(_base_link) == 0)
        return;

    if(!_task->setBaseLink(new_base_link))
    {
        return;
    }

    setBaseLink(new_base_link);


    _menu_handler.setCheckState(_base_link_entry_active, interactive_markers::MenuHandler::UNCHECKED );
    _base_link_entry_active = entry_id;
    _menu_handler.setCheckState(_base_link_entry_active, interactive_markers::MenuHandler::CHECKED );

    _menu_handler.reApply(_server);
    _server.applyChanges();

}

void CartesianMarker::_activatePositionFeedBack(const bool is_active)
{
    if(is_active){
        _position_feedback_active = -1;
        _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::CHECKED);}
    else{
        _position_feedback_active = 1;
        _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);}

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::activatePositionFeedBack(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    _position_feedback_active *= -1;

    if(_position_feedback_active == 1)
    {
        _menu_handler.setCheckState(_position_feedback_is_active_entry,
                                    interactive_markers::MenuHandler::UNCHECKED);

        _task->setControlMode(ControlType::Velocity);
    }
    else if(_position_feedback_active == -1)
    {
        _menu_handler.setCheckState(_position_feedback_is_active_entry,
                                    interactive_markers::MenuHandler::CHECKED);

        _task->setControlMode(ControlType::Position);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::_activateTask(const bool is_active)
{
    if(is_active)
    {
        _task_active = -1;
        _menu_handler.setCheckState(_task_is_active_entry,
                                    interactive_markers::MenuHandler::CHECKED);
    }
    else
    {
        _task_active = 1;
        _menu_handler.setCheckState(_task_is_active_entry,
                                    interactive_markers::MenuHandler::UNCHECKED);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::activateTask(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    _task_active *= -1;

    if(_task_active == 1)
    {
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);
        _task->setActivationState(ActivationState::Disabled);
    }
    else if(_task_active == -1)
    {
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED);
        _task->setActivationState(ActivationState::Enabled);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::setContinuousCtrl(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    _is_continuous *= -1;

    if(_is_continuous == 1)
    {
        _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::UNCHECKED);
        _menu_handler.setVisible(_way_point_entry, true);
        _waypoints.clear();
        _T.clear();
        setContinuous(_req,_res);
    }
    else if(_is_continuous == -1)
    {
        _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::CHECKED);
        _menu_handler.setVisible(_way_point_entry, false);
        _waypoints.clear();
        _T.clear();
        setTrj(_req, _res);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

bool CartesianMarker::setContinuous(EmptySrv::Request::ConstSharedPtr req,
                                    EmptySrv::Response::SharedPtr res)
{
    clearMarker(req, res);
    spawnMarker(req,res);
    return true;
}

bool CartesianMarker::setTrj(EmptySrv::Request::ConstSharedPtr req,
                             EmptySrv::Response::SharedPtr res)
{
    clearMarker(req, res);
    spawnMarker(req, res);
    return true;
}

void CartesianMarker::setControlGlobalLocal(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    _control_type *= -1;

    if(_control_type == 1)
    {
        _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::UNCHECKED);
        setLocal(_req,_res);
    }
    else if(_control_type == -1)
    {
        _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::CHECKED);
        setGlobal(_req, _res);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

bool CartesianMarker::setGlobal(EmptySrv::Request::ConstSharedPtr req,
                                EmptySrv::Response::SharedPtr res)
{
    namespace pl = std::placeholders;

    _int_marker.pose = tf2::toMsg(_actual_pose);

    for(unsigned int i = 1; i < _int_marker.controls.size(); ++i)
    {
        _int_marker.controls[i].orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
    }

    _server.insert(_int_marker, std::bind(std::mem_fn(&CartesianMarker::MarkerFeedback), this, pl::_1));

    _server.applyChanges();

    return true;
}

bool CartesianMarker::setLocal(EmptySrv::Request::ConstSharedPtr req,
                               EmptySrv::Response::SharedPtr res)
{
    namespace pl = std::placeholders;

    _int_marker.pose = tf2::toMsg(_actual_pose);

    for(unsigned int i = 1; i < _int_marker.controls.size(); ++i)
    {
        _int_marker.controls[i].orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::INHERIT;
    }

    _server.insert(_int_marker, std::bind(std::mem_fn(&CartesianMarker::MarkerFeedback), this, pl::_1));

    _server.applyChanges();

    return true;
}

bool CartesianMarker::spawnMarker(EmptySrv::Request::ConstSharedPtr req,
                                  EmptySrv::Response::SharedPtr res)
{
    namespace pl = std::placeholders;

    if(_server.empty())
    {
        _start_pose = getRobotActualPose();

        _actual_pose = _start_pose;

        _int_marker.pose = tf2::toMsg(_start_pose);

        _server.insert(_int_marker,
                       std::bind(std::mem_fn(&CartesianMarker::MarkerFeedback), this, pl::_1));

        _menu_handler.apply(_server, _int_marker.name);

        _server.applyChanges();
    }

    return true;
}

bool CartesianMarker::clearMarker(EmptySrv::Request::ConstSharedPtr req,
                                  EmptySrv::Response::SharedPtr res)
{
    if(!_server.empty())
    {
        _server.erase(_int_marker.name);
        _server.applyChanges();
    }
    return true;
}

void CartesianMarker::createInteractiveMarkerControl(const double qw, const double qx, const double qy, const double qz,
                                                     const unsigned int interaction_mode)
{
    using visualization_msgs::msg::InteractiveMarkerControl;

    _control.orientation.w = qw;
    _control.orientation.x = qx;
    _control.orientation.y = qy;
    _control.orientation.z = qz;

    if(interaction_mode == InteractiveMarkerControl::MOVE_ROTATE_3D)
    {
        _control.name = "rotate_x";
        _control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        _int_marker.controls.push_back(_control);
        _control.name = "move_x";
        _control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else if(interaction_mode == InteractiveMarkerControl::MOVE_3D)
    {
        _control.name = "move_x";
        _control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else if(interaction_mode == InteractiveMarkerControl::ROTATE_3D)
    {
        _control.name = "rotate_x";
        _control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else throw std::invalid_argument("Invalid interaction mode!");
}

void CartesianMarker::MakeMarker(const std::string &distal_link,
                                 const std::string &base_link,
                                 bool fixed,
                                 unsigned int interaction_mode,
                                 bool show)
{
    namespace pl = std::placeholders;

    RCLCPP_INFO(_node->get_logger(), "Creating marker %s -> %s\n", base_link.c_str(), distal_link.c_str());
    _int_marker.header.frame_id = _tf_prefix+base_link;
    _int_marker.scale = 0.5;

    _int_marker.name = distal_link;
    _int_marker.description = "";

    // insert STL
    makeSTLControl(_int_marker);

    _int_marker.controls[0].interaction_mode = interaction_mode;

    if(show)
    {
        createInteractiveMarkerControl(1, 1, 0, 0, interaction_mode);
        createInteractiveMarkerControl(1, 0, 1, 0, interaction_mode);
        createInteractiveMarkerControl(1, 0, 0, 1, interaction_mode);
    }

    _int_marker.pose = tf2::toMsg(_start_pose);

    _server.insert(_int_marker,
                   std::bind(std::mem_fn(&CartesianMarker::MarkerFeedback), this, pl::_1));
}

void CartesianMarker::MarkerFeedback(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{
    
    //In base_link
    tf2::fromMsg(feedback->pose, _actual_pose);

    if(_is_continuous == -1)
    {
        _task->setPoseReference(_actual_pose);
    }
    
}

visualization_msgs::msg::InteractiveMarkerControl&
CartesianMarker::makeSTLControl(visualization_msgs::msg::InteractiveMarker &msg )
{
    _control2.always_visible = true;

    if(_use_mesh && _urdf.getLink(_distal_link) != NULL)
        _control2.markers.push_back( makeSTL(msg) );
    else
        _control2.markers.push_back( makeSphere(msg) );

    msg.controls.push_back( _control2 );

    return msg.controls.back();
}

visualization_msgs::msg::Marker CartesianMarker::makeSphere( visualization_msgs::msg::InteractiveMarker &msg )
{
    _marker.type = visualization_msgs::msg::Marker::SPHERE;
    _marker.scale.x = msg.scale * 0.45;
    _marker.scale.y = msg.scale * 0.45;
    _marker.scale.z = msg.scale * 0.45;
    _marker.color.r = 0.5;
    _marker.color.g = 0.5;
    _marker.color.b = 1.5;
    _marker.color.a = 1.0;

    return _marker;
}

namespace
{
Eigen::Affine3d toEigen(urdf::Pose pose)
{
    Eigen::Quaterniond q;
    pose.rotation.getQuaternion(q.x(), q.y(), q.z(), q.w());

    Eigen::Affine3d ret;
    ret.translation() << pose.position.x, pose.position.y, pose.position.z;
    ret.linear() = q.toRotationMatrix();

    return ret;
}
}

visualization_msgs::msg::Marker CartesianMarker::makeSTL(visualization_msgs::msg::InteractiveMarker& msg)
{
    using visualization_msgs::msg::Marker;


    auto link = _urdf.getLink(_distal_link);
    auto controlled_link = link;

    Eigen::Affine3d T;
    T.setIdentity();

    while(!link->visual)
    {
        if(!link->parent_joint)
        {
            XBot::Logger::warning("Unable to find mesh for link %s \n", _distal_link.c_str());
            return makeSphere(msg);
        }
        link = _urdf.getLink(link->parent_joint->parent_link_name);
    }
    
    T = getPose(controlled_link->name, link->name);

    Eigen::Affine3d T_marker = ::toEigen(link->visual->origin);
    T = T*T_marker;

    _marker.pose = tf2::toMsg(T);

    _marker.color.r = 0.5;
    _marker.color.g = 0.5;
    _marker.color.b = 0.5;

    if(link->visual->geometry->type == urdf::Geometry::MESH)
    {
        _marker.type = Marker::MESH_RESOURCE;

        auto mesh =
            std::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

        _marker.mesh_resource = mesh->filename;
        _marker.scale.x = mesh->scale.x;
        _marker.scale.y = mesh->scale.y;
        _marker.scale.z = mesh->scale.z;
    }
    else if(link->visual->geometry->type == urdf::Geometry::BOX)
    {
        _marker.type = Marker::CUBE;

        auto mesh =
                std::static_pointer_cast<urdf::Box>(link->visual->geometry);

        _marker.scale.x = mesh->dim.x;
        _marker.scale.y = mesh->dim.y;
        _marker.scale.z = mesh->dim.z;

    }
    else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
    {
        _marker.type = Marker::CYLINDER;

        auto mesh =
                std::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);

        _marker.scale.x = _marker.scale.y = mesh->radius;
        _marker.scale.z = mesh->length;
    }
    else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
    {
        _marker.type = Marker::SPHERE;

        auto mesh =
                std::static_pointer_cast<urdf::Sphere>(link->visual->geometry);

        _marker.scale.x = _marker.scale.y = _marker.scale.z = 2.*mesh->radius;
    }

    _marker.color.a = .9;

    return _marker;
}

Eigen::Affine3d CartesianMarker::getRobotActualPose()
{
    return getPose(_base_link, _distal_link);
}

Eigen::Affine3d CartesianMarker::getPose(const std::string& base_link, const std::string& distal_link)
{

    Eigen::Affine3d T;
    T.setIdentity();

    try
    {
        auto tf = _tf_buffer->lookupTransform(_tf_prefix + base_link,
                                              _tf_prefix + distal_link,
                                              tf2::TimePointZero,
                                              1s);

        T.matrix() = tf2::transformToEigen(tf.transform).matrix();

    }
    catch (tf2::TransformException ex)
    {
        RCLCPP_ERROR_STREAM(_node->get_logger(), ex.what());
    }

    return T;
}

void XBot::Cartesian::CartesianMarker::setBaseLink(std::string base_link)
{
    _base_link = base_link;
    _int_marker.header.frame_id = _tf_prefix + base_link;

    clearMarker(_req, _res);
    spawnMarker(_req, _res);
}

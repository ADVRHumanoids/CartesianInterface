#include <cartesian_interface/markers/CartesianMarker.h>
#include <cartesian_interface/SetTaskInfo.h>
#include <cartesian_interface/GetTaskInfo.h>
#include <std_srvs/SetBool.h>
#include <numeric> 
#include <XBotInterface/RtLog.hpp>
#include <tf_conversions/tf_kdl.h>

#define SECS 5

using namespace XBot::Cartesian;

CartesianMarker::CartesianMarker(const std::string &base_link,
                                 const std::string &distal_link,
                                 const urdf::Model &robot_urdf,
                                 const unsigned int control_type,
                                 std::string tf_prefix,
                                 const bool use_mesh
                                ):
    _base_link(base_link),
    _distal_link(distal_link),
    _urdf(robot_urdf),
    _server(distal_link + "_Cartesian_marker_server"),
    _tf_prefix(tf_prefix),
    _menu_entry_counter(0),
    _control_type(1),_is_continuous(1), _task_active(-1), _position_feedback_active(-1),
    _waypoint_action_client("cartesian/" + distal_link + "/reach", true),
    _nh("cartesian"),
    _use_mesh(use_mesh)
{
    _urdf.getLinks(_links);

    _start_pose = getRobotActualPose();
    _actual_pose = _start_pose;

    MakeMarker(_distal_link, _base_link, false, control_type, true);

    MakeMenu();

    _server.applyChanges();

    _clear_service = _nh.advertiseService(_int_marker.name + "/clear_marker", &CartesianMarker::clearMarker, this);
    _spawn_service = _nh.advertiseService(_int_marker.name + "/spawn_marker", &CartesianMarker::spawnMarker, this);

    _set_properties_service_client = _nh.serviceClient<cartesian_interface::SetTaskInfo>(_distal_link + "/set_task_properties");
    _get_properties_service_client = _nh.serviceClient<cartesian_interface::GetTaskInfo>(_distal_link + "/get_task_properties");
    
    _set_properties_service_client.waitForExistence();
    _get_properties_service_client.waitForExistence();

//    _global_service = _nh.advertiseService("setGlobal_"+_int_marker.name, &CartesianMarker::setGlobal, this);
//    _local_service = _nh.advertiseService("setLocal_"+_int_marker.name, &CartesianMarker::setLocal, this);

    std::string topic_name = distal_link + "/reference";
    _ref_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(topic_name, 1);

    topic_name = distal_link + "/wp";
    _way_points_pub = _nh.advertise<geometry_msgs::PoseArray>(topic_name, 1, true);
    
    _waypoint_action_client.waitForServer();

}

CartesianMarker::~CartesianMarker()
{

}

void CartesianMarker::MakeMenu()
{
    _menu_entry_counter = 0;

    _reset_marker_entry = _menu_handler.insert("Reset Marker",boost::bind(boost::mem_fn(&CartesianMarker::resetMarker),
                            this, _1));
    _menu_entry_counter++;

    _global_control_entry = _menu_handler.insert("Global Ctrl",boost::bind(boost::mem_fn(&CartesianMarker::setControlGlobalLocal),
                            this, _1));
    _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::UNCHECKED);
    _menu_entry_counter++;

    _continuous_control_entry = _menu_handler.insert("Continuous Ctrl",boost::bind(boost::mem_fn(&CartesianMarker::setContinuousCtrl),
                                                                                   this, _1));
    _menu_handler.setCheckState(_continuous_control_entry, interactive_markers::MenuHandler::UNCHECKED);
    _menu_entry_counter++;

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
        _T_last = _menu_handler.insert( _T_entry, s.str(),
            boost::bind(boost::mem_fn(&CartesianMarker::wayPointCallBack),
                        this, _1));
        _menu_entry_counter++;
        _menu_handler.setCheckState(_T_last, interactive_markers::MenuHandler::UNCHECKED );
    }
    _reset_all_way_points_entry = _menu_handler.insert(_way_point_entry, "Reset All",boost::bind(boost::mem_fn(&CartesianMarker::resetAllWayPoints),
                                                                                                 this, _1));
    _menu_entry_counter++;
    _reset_last_way_point_entry = _menu_handler.insert(_way_point_entry, "Reset Last",boost::bind(boost::mem_fn(&CartesianMarker::resetLastWayPoints),
                                                                                                 this, _1));
    _menu_entry_counter++;

    _send_way_points_entry = _menu_handler.insert(_way_point_entry, "Send",boost::bind(boost::mem_fn(&CartesianMarker::sendWayPoints),
                                                                                                 this, _1));
    _menu_entry_counter++;

    _properties_entry = _menu_handler.insert("Properties");
    _menu_entry_counter++;
    _task_is_active_entry = _menu_handler.insert(_properties_entry, "Task Active",boost::bind(boost::mem_fn(&CartesianMarker::activateTask),
                                                                                this, _1));
    _menu_entry_counter++;
    _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED );
    _position_feedback_is_active_entry = _menu_handler.insert(_properties_entry, "Position FeedBack Active",boost::bind(boost::mem_fn(&CartesianMarker::activatePositionFeedBack),
                                                                                                      this, _1));
    _menu_entry_counter++;
    _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::CHECKED );




    _base_link_entry = _menu_handler.insert(_properties_entry, "Base Link");
    _menu_entry_counter++;
    for(unsigned int i = 0; i < _links.size(); ++i)
    {
        if(_distal_link != _links.at(i)->name)
        {
            interactive_markers::MenuHandler::EntryHandle link_entry = _menu_handler.insert(_base_link_entry,
                _links.at(i)->name, boost::bind(boost::mem_fn(&CartesianMarker::changeBaseLink), this, _1));

            _menu_entry_counter++;

            if(_base_link.compare("world_odom") == 0 && (_links.at(i)->name).compare("world") == 0){
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


    _menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    _menu_control.always_visible = true;

    _int_marker.controls.push_back(_menu_control);

    _menu_handler.apply(_server, _int_marker.name);
}

void CartesianMarker::sendWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    if(!_waypoints.empty())
    {
        ///QUI INSERISCI IL TUO SPORCO CODICE///
        cartesian_interface::ReachPoseGoal goal;
        goal.frames = _waypoints;
        std::partial_sum(_T.begin(), _T.end(), _T.begin());
        goal.time = _T;
        _waypoint_action_client.sendGoal(goal);
        ///////////////////////////////////////

        _waypoints.clear();
        _T.clear();
        publishWP(_waypoints);
    }
}

void CartesianMarker::resetLastWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    _T.pop_back();
    _waypoints.pop_back();
    clearMarker(_req, _res);

    ROS_INFO("RESET LAST WAYPOINT!");

    if(_waypoints.empty())
        spawnMarker(_req, _res);
    else{
        if(_server.empty())
        {
            tf::poseMsgToKDL(_waypoints.back(),_start_pose);
            //tf::PoseMsgToKDL(_waypoints.back(),_start_pose);
            _actual_pose = _start_pose;

            KDLFrameToVisualizationPose(_start_pose, _int_marker);

            _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback),this, _1));
            _menu_handler.apply(_server, _int_marker.name);
            _server.applyChanges();
        }
    }
    publishWP(_waypoints);
}

void CartesianMarker::resetAllWayPoints(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    _T.clear();
    _waypoints.clear();
    resetMarker(feedback);
    publishWP(_waypoints);
    ROS_INFO("RESETTING ALL WAYPOINTS!");
}

void CartesianMarker::resetMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    cartesian_interface::GetTaskInfo srv;
    _get_properties_service_client.call(srv);

    if(srv.response.control_mode.compare("Disabled") == 0)
        _activateTask(false);
    else
    {
        _activateTask(true);
        if(srv.response.control_mode.compare("Position") == 0)
            _activatePositionFeedBack(true);
        else
            _activatePositionFeedBack(false);
    }


    if(srv.response.base_link.compare(_base_link) != 0)
    {
        visualization_msgs::InteractiveMarkerFeedbackPtr feedback;
        feedback = boost::make_shared<visualization_msgs::InteractiveMarkerFeedback>();
        feedback->menu_entry_id = _map_link_entry[srv.response.base_link];
        changeBaseLink(feedback);
    }


    clearMarker(_req, _res);
    spawnMarker(_req, _res);
}

void CartesianMarker::publishWP(const std::vector<geometry_msgs::Pose>& wps)
{
    geometry_msgs::PoseArray msg;
    for(unsigned int i = 0; i < wps.size(); ++i)
        msg.poses.push_back(wps[i]);

    msg.header.frame_id = _tf_prefix+_base_link;
    msg.header.stamp = ros::Time::now();

    _way_points_pub.publish(msg);
}

void CartesianMarker::wayPointCallBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    //In base_link
    tf::poseMsgToKDL(feedback->pose, _actual_pose);
    double qx,qy,qz,qw;
    _actual_pose.M.GetQuaternion(qx,qy,qz,qw);

    double T = double(feedback->menu_entry_id-offset_menu_entry);


    if(_is_continuous == 1)
    {
        ROS_INFO("\n %s set waypoint @: \n pos = [%f, %f, %f],\n orient = [%f, %f, %f, %f],\n of %.1f secs",
                 _int_marker.name.c_str(),
                 _actual_pose.p.x(), _actual_pose.p.y(), _actual_pose.p.z(),
                 qx,qy,qz,qw, T);

        _waypoints.push_back(feedback->pose);
        _T.push_back(T);

        publishWP(_waypoints);

    }
}

void CartesianMarker::changeBaseLink(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    int entry_id = feedback->menu_entry_id;

    std::string new_base_link;
    _menu_handler.getTitle(entry_id, new_base_link);

    if(new_base_link.compare("world") == 0)
        new_base_link = "world_odom";


    if(new_base_link.compare(_base_link) == 0)
        return;

    cartesian_interface::SetTaskInfo srv;
    srv.request.base_link = new_base_link;
    if(!_set_properties_service_client.call(srv) || srv.response.success == false){
        
        ROS_WARN("%s", srv.response.message.c_str());
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

void CartesianMarker::activatePositionFeedBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    cartesian_interface::SetTaskInfo srv;

    _position_feedback_active *= -1;
    if(_position_feedback_active == 1)
    {
        _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);
        srv.request.control_mode = "Velocity";
        _set_properties_service_client.call(srv);
    }
    else if(_position_feedback_active == -1)
    {
        _menu_handler.setCheckState(_position_feedback_is_active_entry, interactive_markers::MenuHandler::CHECKED);
        srv.request.control_mode = "Position";
        _set_properties_service_client.call(srv);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::_activateTask(const bool is_active)
{
    if(is_active){
        _task_active = -1;
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED);}
    else{
        _task_active = 1;
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);}

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::activateTask(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    cartesian_interface::SetTaskInfo srv;

    _task_active *= -1;
    if(_task_active == 1)
    {
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::UNCHECKED);
        srv.request.control_mode = "Disabled";
        _set_properties_service_client.call(srv);
    }
    else if(_task_active == -1)
    {
        _menu_handler.setCheckState(_task_is_active_entry, interactive_markers::MenuHandler::CHECKED);
        srv.request.control_mode = "Position";
        _set_properties_service_client.call(srv);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

void CartesianMarker::setContinuousCtrl(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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

bool CartesianMarker::setContinuous(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    clearMarker(req, res);
    spawnMarker(req,res);
}

bool CartesianMarker::setTrj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    clearMarker(req, res);
    spawnMarker(req,res);
}

void CartesianMarker::setControlGlobalLocal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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

bool CartesianMarker::setGlobal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    KDLFrameToVisualizationPose(_actual_pose, _int_marker);

    for(unsigned int i = 1; i < _int_marker.controls.size(); ++i)
        _int_marker.controls[i].orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback), this, _1));
    _server.applyChanges();
    return true;
}

bool CartesianMarker::setLocal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    KDLFrameToVisualizationPose(_actual_pose, _int_marker);

    for(unsigned int i = 1; i < _int_marker.controls.size(); ++i)
        _int_marker.controls[i].orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
    _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback), this, _1));
    _server.applyChanges();
    return true;
}

bool CartesianMarker::spawnMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    if(_server.empty())
    {
        _start_pose = getRobotActualPose();
        _actual_pose = _start_pose;

        KDLFrameToVisualizationPose(_start_pose, _int_marker);

        _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback),this, _1));
        _menu_handler.apply(_server, _int_marker.name);
        _server.applyChanges();
    }
    return true;
}

bool CartesianMarker::clearMarker(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
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
    _control.orientation.w = qw;
    _control.orientation.x = qx;
    _control.orientation.y = qy;
    _control.orientation.z = qz;
    if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D)
    {
        _control.name = "rotate_x";
        _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        _int_marker.controls.push_back(_control);
        _control.name = "move_x";
        _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D)
    {
        _control.name = "move_x";
        _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else if(interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D)
    {
        _control.name = "rotate_x";
        _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        _int_marker.controls.push_back(_control);
    }
    else throw std::invalid_argument("Invalid interaction mode!");
}

void CartesianMarker::MakeMarker(const std::string &distal_link, const std::string &base_link,
                            bool fixed, unsigned int interaction_mode, bool show)
{
    ROS_INFO("Creating marker %s -> %s\n", base_link.c_str(), distal_link.c_str());
    _int_marker.header.frame_id = _tf_prefix+base_link;
    _int_marker.scale = 0.5;

    _int_marker.name = distal_link;
    _int_marker.description = "";

    // insert STL
    makeSTLControl(_int_marker);


    _int_marker.controls[0].interaction_mode = interaction_mode;


    if(show)
    {
        createInteractiveMarkerControl(1,1,0,0,interaction_mode);
        createInteractiveMarkerControl(1,0,1,0,interaction_mode);
        createInteractiveMarkerControl(1,0,0,1,interaction_mode);
    }

    KDLFrameToVisualizationPose(_start_pose, _int_marker);

    _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback),this, _1));
}

void CartesianMarker::MarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    
    //In base_link
    tf::poseMsgToKDL(feedback->pose, _actual_pose);
    double qx,qy,qz,qw;
    _actual_pose.M.GetQuaternion(qx,qy,qz,qw);

    if(_is_continuous == -1)
    {
//        std::cout<<_int_marker.name<<" _actual_pose: \n ["<<_actual_pose.p.x()<<" "<<_actual_pose.p.y()<<" "<<_actual_pose.p.z()<<"]"<<std::endl;
//        std::cout<<"["<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<"]"<<std::endl;

        geometry_msgs::PoseStamped msg;
        msg.pose = feedback->pose;
        msg.header = feedback->header;

        _ref_pose_pub.publish(msg);
    }
    
}

visualization_msgs::InteractiveMarkerControl& CartesianMarker::makeSTLControl(visualization_msgs::InteractiveMarker &msg )
{
  _control2.always_visible = true;

  if(_use_mesh && _urdf.getLink(_distal_link) != NULL)
    _control2.markers.push_back( makeSTL(msg) );
  else
    _control2.markers.push_back( makeSphere(msg) );

  msg.controls.push_back( _control2 );

  return msg.controls.back();
}

visualization_msgs::Marker CartesianMarker::makeSphere( visualization_msgs::InteractiveMarker &msg )
{
  _marker.type = visualization_msgs::Marker::SPHERE;
  _marker.scale.x = msg.scale * 0.45;//0.45
  _marker.scale.y = msg.scale * 0.45;
  _marker.scale.z = msg.scale * 0.45;
  _marker.color.r = 0.5;
  _marker.color.g = 0.5;
  _marker.color.b = 1.5;
  _marker.color.a = 1.0;

  return _marker;
}

visualization_msgs::Marker CartesianMarker::makeSTL( visualization_msgs::InteractiveMarker &msg )
{
    boost::shared_ptr<const urdf::Link> link = _urdf.getLink(_distal_link);
    boost::shared_ptr<const urdf::Link> controlled_link = link;

    KDL::Frame T; T.Identity();
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
    KDL::Frame T_marker;
    URDFPoseToKDLFrame(link->visual->origin, T_marker);
    T = T*T_marker;
    KDLFrameToVisualizationPose(T, _marker);

    _marker.color.r = 0.5;
    _marker.color.g = 0.5;
    _marker.color.b = 0.5;

    if(link->visual->geometry->type == urdf::Geometry::MESH)
    {
        _marker.type = visualization_msgs::Marker::MESH_RESOURCE;

        boost::shared_ptr<urdf::Mesh> mesh =
                boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

        _marker.mesh_resource = mesh->filename;
        _marker.scale.x = mesh->scale.x;
        _marker.scale.y = mesh->scale.y;
        _marker.scale.z = mesh->scale.z;
    }
    else if(link->visual->geometry->type == urdf::Geometry::BOX)
    {
        _marker.type = visualization_msgs::Marker::CUBE;

        boost::shared_ptr<urdf::Box> mesh =
                boost::static_pointer_cast<urdf::Box>(link->visual->geometry);

        KDL::Frame T_marker;
        URDFPoseToKDLFrame(link->visual->origin, T_marker);

        _marker.scale.x = mesh->dim.x;
        _marker.scale.y = mesh->dim.y;
        _marker.scale.z = mesh->dim.z;

    }
    else if(link->visual->geometry->type == urdf::Geometry::CYLINDER)
    {
        _marker.type = visualization_msgs::Marker::CYLINDER;

        boost::shared_ptr<urdf::Cylinder> mesh =
                boost::static_pointer_cast<urdf::Cylinder>(link->visual->geometry);

        KDL::Frame T_marker;
        URDFPoseToKDLFrame(link->visual->origin, T_marker);

        _marker.scale.x = _marker.scale.y = mesh->radius;
        _marker.scale.z = mesh->length;
    }
    else if(link->visual->geometry->type == urdf::Geometry::SPHERE)
    {
        _marker.type = visualization_msgs::Marker::SPHERE;

        boost::shared_ptr<urdf::Sphere> mesh =
                boost::static_pointer_cast<urdf::Sphere>(link->visual->geometry);

        KDL::Frame T_marker;
        URDFPoseToKDLFrame(link->visual->origin, T_marker);

        _marker.scale.x = _marker.scale.y = _marker.scale.z = 2.*mesh->radius;
    }

    _marker.color.a = .9;
    return _marker;
}

KDL::Frame CartesianMarker::getRobotActualPose()
{
    for(unsigned int i = 0; i < 10; ++i){
    try{
        ros::Time now = ros::Time::now();
        _listener.waitForTransform(_tf_prefix+_base_link,
                                   _tf_prefix+_distal_link,ros::Time(0),ros::Duration(1.0));

        _listener.lookupTransform(_tf_prefix+_base_link, _tf_prefix+_distal_link,
            ros::Time(0), _transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }}
    KDL::Frame transform_KDL;
    tf::TransformTFToKDL(_transform, transform_KDL);

    return transform_KDL;
}

KDL::Frame CartesianMarker::getPose(const std::string& base_link, const std::string& distal_link)
{

    for(unsigned int i = 0; i < 10; ++i){
    try{
        ros::Time now = ros::Time::now();
        _listener.waitForTransform(_tf_prefix+base_link, _tf_prefix+distal_link,ros::Time(0),ros::Duration(1.0));

        _listener.lookupTransform(_tf_prefix+base_link, _tf_prefix+distal_link,
            ros::Time(0), _transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }}

    KDL::Frame transform_KDL;
    tf::TransformTFToKDL(_transform, transform_KDL);

    return transform_KDL;
}

void XBot::Cartesian::CartesianMarker::setBaseLink(std::string base_link)
{
    _base_link = base_link;
    _int_marker.header.frame_id = _tf_prefix + base_link;

    clearMarker(_req, _res);
    spawnMarker(_req, _res);
}

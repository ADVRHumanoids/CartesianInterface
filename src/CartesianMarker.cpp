#include <cartesian_interface/markers/CartesianMarker.h>

using namespace XBot::Cartesian;

CartesianMarker::CartesianMarker(const std::string &base_link,
                                 const std::string &distal_link,
                                 const urdf::Model &robot_urdf,
                                 std::string tf_prefix
                                ):
    _nh("xbotcore/cartesian"),
    _base_link(base_link),
    _distal_link(distal_link),
    _urdf(robot_urdf),
    _server(distal_link + "_Cartesian_marker_server"),
    _tf_prefix(tf_prefix),
    _menu_entry_counter(0),
    _control_type(1)
{
    _start_pose = getRobotActualPose();

    MakeMarker(_distal_link, _base_link, false,
               visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
               true);

    MakeMenu();

    _server.applyChanges();

    _clear_service = _nh.advertiseService(_int_marker.name + "/clear_marker", &CartesianMarker::clearMarker, this);
    _spawn_service = _nh.advertiseService(_int_marker.name + "/spawn_marker", &CartesianMarker::spawnMarker, this);
//    _global_service = _nh.advertiseService("setGlobal_"+_int_marker.name, &CartesianMarker::setGlobal, this);
//    _local_service = _nh.advertiseService("setLocal_"+_int_marker.name, &CartesianMarker::setLocal, this);

    std::string topic_name = distal_link + "/reference";
    _ref_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>(topic_name, 1);

}

CartesianMarker::~CartesianMarker()
{

}

void CartesianMarker::MakeMenu()
{
    _menu_entry_counter = 0;

    
    _global_control_entry = _menu_handler.insert("Global Ctrl",boost::bind(boost::mem_fn(&CartesianMarker::setControlGlobalLocal),
                            this, _1));
    _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::UNCHECKED);
    _menu_entry_counter++;


    _menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    _menu_control.always_visible = true;

    _int_marker.controls.push_back(_menu_control);

    _menu_handler.apply(_server, _int_marker.name);
}

void CartesianMarker::setControlGlobalLocal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;

    _control_type *= -1;
    if(_control_type == 1)
    {
        _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::UNCHECKED);
        setLocal(req,res);
    }
    else if(_control_type == -1)
    {
        _menu_handler.setCheckState(_global_control_entry, interactive_markers::MenuHandler::CHECKED);
        setGlobal(req, res);
    }

    _menu_handler.reApply(_server);
    _server.applyChanges();
}

bool CartesianMarker::setGlobal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _int_marker.pose.position.x = _actual_pose.p.x();
    _int_marker.pose.position.y = _actual_pose.p.y();
    _int_marker.pose.position.z = _actual_pose.p.z();
    double qx,qy,qz,qw; _actual_pose.M.GetQuaternion(qx,qy,qz,qw);
    _int_marker.pose.orientation.x = qx;
    _int_marker.pose.orientation.y = qy;
    _int_marker.pose.orientation.z = qz;
    _int_marker.pose.orientation.w = qw;
    for(unsigned int i = 1; i < _int_marker.controls.size(); ++i)
        _int_marker.controls[i].orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback), this, _1));
    _server.applyChanges();
    return true;
}

bool CartesianMarker::setLocal(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _int_marker.pose.position.x = _actual_pose.p.x();
    _int_marker.pose.position.y = _actual_pose.p.y();
    _int_marker.pose.position.z = _actual_pose.p.z();
    double qx,qy,qz,qw; _actual_pose.M.GetQuaternion(qx,qy,qz,qw);
    _int_marker.pose.orientation.x = qx;
    _int_marker.pose.orientation.y = qy;
    _int_marker.pose.orientation.z = qz;
    _int_marker.pose.orientation.w = qw;
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
        _int_marker.pose.position.x = _start_pose.p.x();
        _int_marker.pose.position.y = _start_pose.p.y();
        _int_marker.pose.position.z = _start_pose.p.z();
        double qx,qy,qz,qw; _start_pose.M.GetQuaternion(qx,qy,qz,qw);
        _int_marker.pose.orientation.x = qx;
        _int_marker.pose.orientation.y = qy;
        _int_marker.pose.orientation.z = qz;
        _int_marker.pose.orientation.w = qw;

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

void CartesianMarker::MakeMarker(const std::string &distal_link, const std::string &base_link,
                            bool fixed, unsigned int interaction_mode, bool show_6dof)
{
    _int_marker.header.frame_id = _tf_prefix+base_link;
    _int_marker.scale = 0.5;

    _int_marker.name = distal_link;
    _int_marker.description = "";

    // insert STL
    makeSTLControl(_int_marker);

    _int_marker.controls[0].interaction_mode = interaction_mode;

    if ( fixed )
    {
      //int_marker.name += "_fixed";
      _int_marker.description += "";
      _control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        //int_marker.name += "_" + mode_text;
        _int_marker.description = "";
    }

    if(show_6dof)
    {
      _control.orientation.w = 1;
      _control.orientation.x = 1;
      _control.orientation.y = 0;
      _control.orientation.z = 0;
      _control.name = "rotate_x";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      _int_marker.controls.push_back(_control);
      _control.name = "move_x";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      _int_marker.controls.push_back(_control);

      _control.orientation.w = 1;
      _control.orientation.x = 0;
      _control.orientation.y = 1;
      _control.orientation.z = 0;
      _control.name = "rotate_z";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      _int_marker.controls.push_back(_control);
      _control.name = "move_z";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      _int_marker.controls.push_back(_control);

      _control.orientation.w = 1;
      _control.orientation.x = 0;
      _control.orientation.y = 0;
      _control.orientation.z = 1;
      _control.name = "rotate_y";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      _int_marker.controls.push_back(_control);
      _control.name = "move_y";
      _control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      _int_marker.controls.push_back(_control);
    }

    _int_marker.pose.position.x = _start_pose.p.x();
    _int_marker.pose.position.y = _start_pose.p.y();
    _int_marker.pose.position.z = _start_pose.p.z();
    double qx,qy,qz,qw; _start_pose.M.GetQuaternion(qx,qy,qz,qw);
    _int_marker.pose.orientation.x = qx;
    _int_marker.pose.orientation.y = qy;
    _int_marker.pose.orientation.z = qz;
    _int_marker.pose.orientation.w = qw;

    _server.insert(_int_marker, boost::bind(boost::mem_fn(&CartesianMarker::MarkerFeedback),this, _1));
}

void CartesianMarker::MarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    
    //In base_link
    tf::poseMsgToKDL(feedback->pose, _actual_pose);
    double qx,qy,qz,qw;
    _actual_pose.M.GetQuaternion(qx,qy,qz,qw);

    
    geometry_msgs::PoseStamped msg;
    msg.pose = feedback->pose;
    msg.header = feedback->header;
    
    _ref_pose_pub.publish(msg);
    
}

visualization_msgs::InteractiveMarkerControl& CartesianMarker::makeSTLControl(visualization_msgs::InteractiveMarker &msg )
{
  _control2.always_visible = true;
  _control2.markers.push_back( makeSTL(msg) );
  msg.controls.push_back( _control2 );

  return msg.controls.back();
}

visualization_msgs::Marker CartesianMarker::makeSTL( visualization_msgs::InteractiveMarker &msg )
{
    boost::shared_ptr<const urdf::Link> link = _urdf.getLink(_distal_link);
    boost::shared_ptr<const urdf::Link> controlled_link = link;

    KDL::Frame T; T.Identity();
    while(!link->visual)
            link = _urdf.getLink(link->parent_joint->parent_link_name);
    T = getPose(controlled_link->name, link->name);



    if(link->visual->geometry->type == urdf::Geometry::MESH)
    {
        _marker.type = visualization_msgs::Marker::MESH_RESOURCE;

        boost::shared_ptr<urdf::Mesh> mesh =
                boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);

        _marker.mesh_resource = mesh->filename;


        KDL::Frame T_marker;
        T_marker.p.x(link->visual->origin.position.x);
        T_marker.p.y(link->visual->origin.position.y);
        T_marker.p.z(link->visual->origin.position.z);
        T_marker.M = T_marker.M.Quaternion(link->visual->origin.rotation.x,
                              link->visual->origin.rotation.y,
                              link->visual->origin.rotation.z,
                              link->visual->origin.rotation.w);

        T = T*T_marker;
        _marker.pose.position.x = T.p.x();
        _marker.pose.position.y = T.p.y();
        _marker.pose.position.z = T.p.z();
        double qx,qy,qz,qw; T.M.GetQuaternion(qx,qy,qz,qw);
        _marker.pose.orientation.x = qx;
        _marker.pose.orientation.y = qy;
        _marker.pose.orientation.z = qz;
        _marker.pose.orientation.w = qw;

        _marker.color.r = 0.5;
        _marker.color.g = 0.5;
        _marker.color.b = 0.5;

        _marker.scale.x = mesh->scale.x;
        _marker.scale.y = mesh->scale.y;
        _marker.scale.z = mesh->scale.z;
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
    KDL::Frame transform_KDL; transform_KDL = transform_KDL.Identity();
    transform_KDL.p.x(_transform.getOrigin().x());
    transform_KDL.p.y(_transform.getOrigin().y());
    transform_KDL.p.z(_transform.getOrigin().z());

    transform_KDL.M = transform_KDL.M.Quaternion(
                _transform.getRotation().getX(), _transform.getRotation().getY(),
                _transform.getRotation().getZ(), _transform.getRotation().getW()
                );

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
    KDL::Frame transform_KDL; transform_KDL = transform_KDL.Identity();
    transform_KDL.p.x(_transform.getOrigin().x());
    transform_KDL.p.y(_transform.getOrigin().y());
    transform_KDL.p.z(_transform.getOrigin().z());

    transform_KDL.M = transform_KDL.M.Quaternion(
                _transform.getRotation().getX(), _transform.getRotation().getY(),
                _transform.getRotation().getZ(), _transform.getRotation().getW()
                );

    return transform_KDL;
}

void XBot::Cartesian::CartesianMarker::setBaseLink(std::string base_link)
{
    _base_link = base_link;
    _int_marker.header.frame_id = _tf_prefix + base_link;
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    
    clearMarker(req, res);
    spawnMarker(req, res);
}
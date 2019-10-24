#include <cartesian_interface/problem/Task.h>
#include <cartesian_interface/open_sot/TaskInterface.h>
#include <OpenSoT/constraints/velocity/SelfCollisionAvoidance.h>
#include <boost/make_shared.hpp>
#include <visualization_msgs/Marker.h>
#include <ros/node_handle.h>


using namespace XBot::Cartesian;

/* (1) Define struct that contains the description of the task/constraint, 
 * to be parsed from the problem description YAML file 
 */
struct SelfCollisionAvoidance : public ConstraintDescription // inherit from [Constraint/Task]Description!
{
    std::list<std::string> contact_links;
    double safety_margin;
    std::string base_link;
    double detection_threshold;
    double link_pair_threshold;
    double bound_scaling;

    SelfCollisionAvoidance(std::list<std::string> contact_links);
};

SelfCollisionAvoidance::SelfCollisionAvoidance(std::list<std::string> arg_contact_links):
    ConstraintDescription("SelfCollisionAvoidance"),  // task type (specified by user in YAML file)
    contact_links(arg_contact_links),
    safety_margin(.01)
{
}

/* (2) Define a factory function that dynamically allocates the struct defined in (1)
 * given a YAML::Node and an XBot::ModelInterface::ConstPtr.
 * The function must be declared as 'extern "C"' in order to disable mangling.
 * The function must be named #TASKTYPE#[Task/Constraint]DescriptionFactory.
 * This is the place where you parse the yaml node.
 */
extern "C" ConstraintDescription * SelfCollisionAvoidanceConstraintDescriptionFactory(YAML::Node task_node,
                                                                          XBot::ModelInterface::ConstPtr model)
{
    std::list<std::string> contact_links;
    
    if(task_node["contact_links"])
    {
        for(auto cl : task_node["contact_links"])
        {
            contact_links.push_back(cl.as<std::string>());
        }
    }
    else
    {
        throw std::runtime_error("Missing mandatory node 'contact_links' in SelfCollisionAvoidance task");
    }
    
    SelfCollisionAvoidance * task_desc = new SelfCollisionAvoidance(contact_links);
    
    if(task_node["base_link"])
      task_desc->base_link = task_node["base_link"].as<std::string>();
    else
      task_desc->base_link = "world";

    if(task_node["detection_threshold"])
      task_desc->detection_threshold = task_node["detection_threshold"].as<double>();
    else
      task_desc->detection_threshold = std::numeric_limits<double>::infinity();

    if(task_node["link_pair_threshold"])
      task_desc->link_pair_threshold = task_node["link_pair_threshold"].as<double>();
    else
      task_desc->link_pair_threshold = 0.0;

    if(task_node["bound_scaling"])
      task_desc->bound_scaling = task_node["bound_scaling"].as<double>();
    else
      task_desc->bound_scaling = 1.0;
    
    return task_desc;
    
}

/* (3) Define the corresponding SoT::TaskInterface class. Its purpose is to:
 *  - construct the correct OpenSoT task/constr from the [Task/Constraint]Description defined in (1)
 *  - provide setBaseLink, setControlMode, and update functionalities (if any)
 * The constructor must take two arguments as in this example.
 */
class SelfCollisionAvoidanceOpenSot : public SoT::ConstraintInterface
{
    
public:
    
    SelfCollisionAvoidanceOpenSot(ConstraintDescription::Ptr constr_desc,
                      XBot::ModelInterface::ConstPtr model):
        SoT::ConstraintInterface(constr_desc, model),
        _model(model)
    {
        
        auto desc = std::dynamic_pointer_cast<SelfCollisionAvoidance>(constr_desc);
        
        Eigen::VectorXd q;
        model->getJointPosition(q);
        
        _constr = boost::make_shared<OpenSoT::constraints::velocity::SelfCollisionAvoidance>
                                (q,
                                 const_cast<XBot::ModelInterface&>(*model), // HACK non-const model required
                                 desc->base_link,
                                 desc->detection_threshold,
                                 desc->link_pair_threshold,
                                 desc->bound_scaling
                                );
        _vis_pub = _n.advertise<visualization_msgs::Marker>( "self_collision_avoidance", 0 );
    }

    SoT::ConstraintPtr getConstraintPtr() const override
    {
        return _constr;
    }
    
    bool setBaseLink(const std::string & ee_name, const std::string & base_link) override
    {
        return false;
    }
    
    bool setControlMode(const std::string & ee_name, ControlType ctrl_mode) override
    {
        return false;
    }

    bool update(const CartesianInterface * ci, double time, double period) override
    {
        /**
          * This part should be done in a dedicated update_ros() in the future
          */
        OpenSoT::constraints::velocity::SelfCollisionAvoidance::Ptr constr = boost::static_pointer_cast<
                OpenSoT::constraints::velocity::SelfCollisionAvoidance>(_constr);
        ch.clear();

        /*if(constr->getConvexHull(ch))
        {
            ch_marker.header.frame_id = "ci/world_odom";
            ch_marker.header.stamp = ros::Time::now();
            ch_marker.ns = "convex_hull";
            ch_marker.id = 0;
            ch_marker.type = visualization_msgs::Marker::LINE_STRIP;
            ch_marker.action = visualization_msgs::Marker::ADD;

            KDL::Vector com;
            _model->getCOM(com);

            geometry_msgs::Point p;
            ch_marker.points.clear();
            for(std::vector<KDL::Vector>::iterator i =ch.begin(); i != ch.end(); ++i){
                p.x = i->x() + com[0];
                p.y = i->y() + com[1];
                p.z = i->z();// + com[2];

                ch_marker.points.push_back(p);
            }

            p.x = ch.begin()->x() + com[0];
            p.y = ch.begin()->y() + com[1];
            p.z = ch.begin()->z();// + com[2];
            ch_marker.points.push_back(p);

            ch_marker.color.a = 1.0;
            ch_marker.color.r = 0.0;
            ch_marker.color.g = 1.0;
            ch_marker.color.b = 0.0;

            ch_marker.scale.x = 0.01;

            _vis_pub.publish(ch_marker);
        }*/

        /** **/

        return true;
    }
    
private:
    
    SoT::ConstraintPtr _constr;
    ros::NodeHandle _n;
    ros::Publisher _vis_pub;
    XBot::ModelInterface::ConstPtr _model;
    std::vector<KDL::Vector> ch;
    visualization_msgs::Marker ch_marker;


    
};

/* (4) Define the factory function for the SoT::[Task/Constraint]Interface as well. */
extern "C" SoT::ConstraintInterface * SelfCollisionAvoidanceOpenSotConstraintFactory(ConstraintDescription::Ptr task_desc,
                                                         XBot::ModelInterface::ConstPtr model)
{
    return new SelfCollisionAvoidanceOpenSot(task_desc, model);
}
